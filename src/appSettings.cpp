#include "appSettings.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include <stddef.h>
    #include <cstring>
#endif

#include "config.h"

AppSettings::AppSettings() :  m_mqtt_server(NULL), m_mqtt_port(KDefaultMqttPort), m_mqtt_user(NULL), m_mqtt_passw(NULL)
{
}
AppSettings::~AppSettings()
{
    clear();
}

void AppSettings::clear()
{
    delete m_mqtt_server;
    m_mqtt_server = NULL;
    m_mqtt_port = KDefaultMqttPort;
    delete m_mqtt_user;
    m_mqtt_user = NULL;
    delete m_mqtt_passw;
    m_mqtt_passw = NULL;
}

void AppSettings::print() const
{
    Serial.println("AppSettings:");
    Serial.print("m_mqtt_server: ");    Serial.println(m_mqtt_server);
    Serial.print("m_mqtt_port: ");      Serial.println(m_mqtt_port);
    Serial.print("m_mqtt_user: ");      Serial.println(m_mqtt_user);
}
bool AppSettings::isValid() const
{
    return (m_mqtt_server != NULL && m_mqtt_user != NULL && m_mqtt_passw != NULL);
}