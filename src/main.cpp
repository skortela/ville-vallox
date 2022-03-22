
//#define NO_GLOBAL_SERIAL
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <WebSerial.h>

#ifdef ENABLE_OTA
#include <ArduinoOTA.h>
#endif

//#include <RemoteDebug.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "vallox_constants.h"
#include "debug.h"
#include "vallox.h"
#include "config.h"
#include "appSettings.h"
#include "eepromStream.h"
#include <EEPROM.h>

#define ST(A) #A
#define STR(A) ST(A)


unsigned long m_lastStatusUpdate = 0;


AsyncWebServer server(80);
//WebSerialClass Serial;

WiFiClient m_espClient;
PubSubClient m_mqttClient(m_espClient);

CVallox m_vallox;
AppSettings m_settings;

unsigned long m_lastData(0);

#ifdef ENABLE_OTA
void initOTA() {
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");

    Serial.print("FreeSketchSpace: ");
    Serial.println(ESP.getFreeSketchSpace());
    
#ifdef OTA_PASSWD
    // Require password
    ArduinoOTA.setPassword(STR(OTA_PASSWD));
#else
    #pragma message "Warning: no OTA password"
    Serial.println("Warning! no OTA password defined");
#endif

    ArduinoOTA.onStart([]() {
        //m_mqttClient.disconnect();
        DBG("OTA updating");
    });
    ArduinoOTA.onEnd([]() {
        DBG("Update finished, rebooting");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        int prog = progress / (total / 100);
        //Serial.printf("Progress: %u%%\r", prog);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        //Serial.printf("Update error [%u]: ", error);
        if (error == OTA_AUTH_ERROR) {DBG("Auth Failed");}
        else if (error == OTA_BEGIN_ERROR) {DBG("Begin Failed");}
        else if (error == OTA_CONNECT_ERROR) {DBG("Connect Failed");}
        else if (error == OTA_RECEIVE_ERROR) {DBG("Receive Failed");}
        else if (error == OTA_END_ERROR) {DBG("End Failed");}
        else {DBG("Unknown error");}
    });
    ArduinoOTA.begin();
}
#endif

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
    DBG("Received Data...");
    String d = "";
    for(int i=0; i < len; i++){
        d += char(data[i]);
    }
    DBG(d.c_str());
    DBG("recvMsg done");
}

void sendState() {
    char* jsonBuffer = m_vallox.toJson();
    DBG("sending state");
    //DBG(jsonBuffer);

    m_mqttClient.publish(KMqttStateTopic, jsonBuffer);
    free(jsonBuffer);
}

void mqtt_reconnect() {
    // Loop until we're reconnected
    while (!m_mqttClient.connected()) {
        if (!m_settings.isValid()) {
            Serial.println("Settings are not valid!");
            return;
        }
        DBG("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ville-vallox-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect

        m_mqttClient.setServer(m_settings.m_mqtt_server, m_settings.m_mqtt_port);

        if (m_mqttClient.connect(clientId.c_str(), m_settings.m_mqtt_user, m_settings.m_mqtt_passw /*"rag@6aX-hjQ-zEUG9wz8ZBK-m"*/, KMqttAvailability, MQTTQOS0, true, KOffline)) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            m_mqttClient.publish(KMqttAvailability, KOnline, true);
            // ... and resubscribe
            m_mqttClient.subscribe(KMqttTopicCommandListen);

            //sendState();
            m_lastStatusUpdate = 0;
        } else {
            Serial.print("failed, rc=");
            Serial.print(m_mqttClient.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void mqtt_message_received(char* topic, byte* payload, unsigned int length)
{
    DBG("Message arrived [%s]", topic);

    const char* cmd = strrchr(topic, '/');
    if (cmd != NULL) {
        cmd++; // adjust one byte
        DBG("cmd: %s", cmd);
    }
    else {
        // invalid topic
        return;
    }

    if (length > 512) {
        // some sanity check, ignore messages with huge payloads
        return;
    }
    
    char buffer[length + 1];
    memcpy(&buffer, payload, length);
    buffer[length] = '\0';

    DBG("payload: %s", &buffer);

    if (strcmp(cmd, "fan_speed") == 0)
    {
        long int speed = (int)strtol(buffer, NULL, 0);
        if (speed == 0 || speed == LONG_MAX || speed == LONG_MIN)
        {
            DBG("Failed to parse fan_speed value!");
        }
        else
        {
            int retVal = m_vallox.setFanSpeed(speed);
            if (retVal != 0)
            {
                DBG("setFanSpeed [ %d ] failed!", speed);
            }
        }
    }
    else if (strcmp(cmd, "takkakytkin") == 0)
    {
        if (strcmp(&buffer[0], "enable") == 0
            || strcmp(&buffer[0], "activate") == 0
            || strcmp(&buffer[0], "on") == 0
            || strcmp(&buffer[0], "1") == 0)
            {
                int retVal = m_vallox.activateTakkakytkin();
                if (retVal != 0)
                {
                    DBG("activateTakkakytkin failed!");
                }
            }
        else {
            DBG("Invalid payload on 'takkakytkin' cmnd");
        }
    }
    else if (strcmp(cmd, "post_heating_temp") == 0)
    {
        long int temp = (int)strtol(buffer, NULL, 0);
        if (temp == 0 || temp == LONG_MAX || temp == LONG_MIN)
        {
            DBG("Failed to parse post_heating_temp value!");
        }
        else
        {
            int retVal = m_vallox.setPostHeatingTemp(temp);
            if (retVal != 0)
            {
                DBG("setPostHeatingTemp [ %d ] failed!", temp);
            }
        }
    }
    else
    {
        DBG("unsupported cmnd received: %s", cmd);
    }
}

void vallox_params_changed()
{
    DBG("vallox_params_changed");
    DBG("millis: %ld", millis());
    m_lastStatusUpdate = 0;
}


bool loadConfig() {
    Serial.println("loadConfig");
    delay(500);
    EepromStream stream;
   
    stream.setUnderlyingData( const_cast<uint8_t*>(EEPROM.getConstDataPtr()), 150);

    if (stream.readInt32() != KConfigValidationMagic)
    {
        Serial.println("no existing conf!");
        // no existing configuration
        return false;
    }
    Serial.println("found conf!");
    delay(500);

    m_settings.clear();
    m_settings.m_mqtt_server = stream.readStringDup();
    m_settings.m_mqtt_port = stream.readInt32();
    m_settings.m_mqtt_user = stream.readStringDup();
    m_settings.m_mqtt_passw = stream.readStringDup();
    m_settings.print();
    Serial.println("conf loaded");
    delay(500);
    return true;
}

void saveConfig() {
    EepromStream stream;
    stream.setUnderlyingData( EEPROM.getDataPtr(), 150);
    
    stream.writeInt32(KConfigValidationMagic);

    stream.writeString(m_settings.m_mqtt_server);
    stream.writeInt32(m_settings.m_mqtt_port);
    stream.writeString(m_settings.m_mqtt_user);
    stream.writeString(m_settings.m_mqtt_passw);

    EEPROM.commit();
}

void resetConfig() {
    // clear first 100 bytes. it's enought to clear first int32 val, but erasing user sensitive data is more secure aproach.
    for (int i=0; i< 100; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
}

void wifiSetup()
{
    char strPort[10];
    sprintf(strPort, "%d", m_settings.m_mqtt_port);

    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length

    AsyncWiFiManagerParameter custom_mqtt_server("server", "mqtt server", m_settings.m_mqtt_server, 40);
    AsyncWiFiManagerParameter custom_mqtt_port("port", "mqtt port", strPort, 6);
    AsyncWiFiManagerParameter custom_mqtt_user("username", "username", m_settings.m_mqtt_user, 40);
    AsyncWiFiManagerParameter custom_mqtt_passw("password", "password", m_settings.m_mqtt_passw, 40);


    DNSServer dns;
    AsyncWiFiManager wifiManager(&server, &dns);

    wifiManager.setCustomHeadElement("<meta charset=\"UTF-8\">");

    //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    //wifiManager.setAPCallback(configModeCallback);
    //set config save notify callback
    //wifiManager.setSaveConfigCallback(saveConfigCallback);

    if (WiFi.SSID() != "")
    {
        Serial.println("Saved SSID found, set timeout for config portal");
        wifiManager.setConfigPortalTimeout(300); // 5 minutes timeout
    }

    //add all your parameters here
    
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_passw);

    //wifiManager.addParameter(&custom_onewire_pin);
   


    if(!wifiManager.autoConnect(KDeviceHostname)) {
        Serial.println("failed to connect and hit timeout");
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(1000);
    }

    Serial.println("autoconnect continue");

     //wifiManager.addParameter(&custom_onewire_pin);
    /*if(wifiManager.server->hasArg("cars")) {
        String value = wifiManager.server->arg("cars");
        Serial.println("value: " + value);
    }*/
    

    m_settings.clear();
    m_settings.m_mqtt_server = strdup(custom_mqtt_server.getValue());
    m_settings.m_mqtt_port = atoi(custom_mqtt_port.getValue());
    m_settings.m_mqtt_user = strdup(custom_mqtt_user.getValue());
    m_settings.m_mqtt_passw = strdup(custom_mqtt_passw.getValue());

    m_settings.print();

    // Save custom params
    saveConfig();
    /*if (m_shouldSaveConfig) {
        // Save custom params
        saveConfig();
        Serial.println("config saved");
    }*/

    bool ok = WiFi.hostname(KDeviceHostname);
    if (!ok) {
        Serial.println("Failed to se hostname!");
    }

    Serial.println("Connected");
    Serial.println("local ip");
    Serial.println(WiFi.localIP());
}

void setup() {
    EEPROM.begin(300);
    // put your setup code here, to run once:
    Serial.begin(115200);

    loadConfig();

    m_vallox.begin(RO, DI);
    m_vallox.setCallback(vallox_params_changed);
    /*mod.begin(9600); // modbus configuration
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(RE,LOW);
    digitalWrite(DE,LOW);*/

    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.msgCallback(recvMsg);
    server.begin();

#ifdef WIFI_SSID

    WiFi.begin(STR(WIFI_SSID),STR(WIFI_PW));

    Serial.print("Connecting to: ");
    Serial.println(STR(WIFI_SSID));
    // Keep checking the connection status until it is connected
    while (WiFi.status() != WL_CONNECTED) {
        delay(10);
    }
#else
    wifiSetup();
#endif

    Serial.println("Connected");
    //Serial.println("local ip");
    //Serial.println(WiFi.localIP());

#ifdef ENABLE_OTA
    initOTA();
#endif

    m_mqttClient.setBufferSize(800);
    m_mqttClient.setCallback(mqtt_message_received);
    
    Serial.println("Started!");
    DBG("Started!");
}

void loop() {
#ifdef ENABLE_OTA
    ArduinoOTA.handle();
#endif

    if (!m_mqttClient.connected()) {
        mqtt_reconnect();
    }
    m_mqttClient.loop();

    m_vallox.loop();

    if (m_lastStatusUpdate == 0 || (unsigned long) (millis() - m_lastStatusUpdate) > KStatusUpdateInterval) {
        // send status update
        sendState();
        m_lastStatusUpdate = millis();
    }
}

