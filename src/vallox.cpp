/*

    Protocol specification:
    https://community.symcon.de/uploads/short-url/fp2ucSqkcPPBqQ4Lc2UWcsJr4Pn.pdf
*/
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <ArduinoJson.h>
#include <climits>

#include "vallox.h"
#include "debug.h"


#include "valloxMqttMappings.h"

const uint8_t KClient_id = 0x22; // select id from 0x21 to 0x2F, 0x21 is usually reserved for first controller


unsigned long elapsed(unsigned long lastMillis) {
    return millis() - lastMillis;
}

/*
int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        DBG("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    //tty.c_cc[VTIME] = 0; // inter-character timer unused
    //tty.c_cc[VMIN]  = 1; // blocking read until 1 character arrives

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcflush(fd, TCIFLUSH);  // clean line
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        DBG("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        DBG("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        DBG("error %d setting term attributes", errno);
    }
}*/

// Get bit status from given byte and index. index starts from 0.
bool get_bit(uint8_t byteval, uint8_t index)
{
    return ((byteval & (1<<index)) != 0);
}

// Set bit to true from given byte and index. index starts from 0.
uint8_t set_bit(uint8_t byteval, uint8_t index)
{
    uint8_t newbyte = byteval;
    newbyte |= 1 << index;
    return newbyte;
}

// Clear bit from given byte and index. index starts from 0.
uint8_t clear_bit(uint8_t byteval, uint8_t index)
{
    uint8_t newbyte = byteval;
    newbyte &= ~(1 << index);
    return newbyte;
}

int getTemp(uint8_t ntcValue)
{
    std::map<uint8_t, int>::const_iterator it = g_ntcMap.find(ntcValue);
    if (it != g_fanSpeed.end())
    {
        return it->second;
    }
    else
        return INT_MIN;
}
uint8_t getNtcValue(int temperature)
{
    std::map<uint8_t, int>::const_iterator it = g_ntcMap.begin();
    while (it != g_ntcMap.end())
    {
        if (it->second == temperature)
            return it->first;
        else
            it++;
    }
    return 0;
}


int getFanSpeed(uint8_t speedHex)
{
    std::map<uint8_t, int>::const_iterator it = g_fanSpeed.find(speedHex);
    if (it != g_fanSpeed.end())
    {
        return it->second;
    }
    else
        return INT_MIN;
}
uint8_t getFanSpeedRaw(int speed)
{
    std::map<uint8_t, int>::const_iterator it = g_fanSpeed.begin();
    while (it != g_fanSpeed.end())
    {
        if (it->second == speed)
            return it->first;
        else
            it++;
    }
    return 0;
}

const char* getParamName(uint8_t param) {
    std::map<uint8_t, const char*>::const_iterator it = g_mqttParamTopic.find(param);
    if (it != g_mqttParamTopic.end()) {
        return it->second;
    }
    return NULL;
}

const char* getParamBitName(uint8_t param, uint8_t bitIndex) {
    std::map<uint8_t, std::map<uint8_t, const char*> >::const_iterator it = g_mqttParamBitTopic.find(param);
    if (it != g_mqttParamBitTopic.end()) {
        std::map<uint8_t, const char*>::const_iterator it2 = it->second.find(bitIndex);
        if (it2 != it->second.end()) {
            return it2->second;
        }
    }
    return NULL;
}

bool is_param_known_bitfield(uint8_t param)
{
    std::map<uint8_t, std::map<uint8_t, const char*> >::const_iterator it = g_mqttParamBitTopic.find(param);
    if (it != g_mqttParamBitTopic.end())
        return true;
    else
        return false;
}

CVallox::CVallox()
{
    setCallback(NULL);
    //mod.begin(9600); // modbus configuration
    
    //mod.setTransmitEnablePin(RE);
}
CVallox::~CVallox()
{
}

void CVallox::begin(int8_t rxPin, int8_t txPin)
{
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(RE,LOW);
    digitalWrite(DE,LOW);

    mod.begin(9600, SWSERIAL_8N1, rxPin, txPin); // RX=26 , TX =27
    mod.setTimeout(3);
}
void CVallox::setCallback(VALLOX_CALLBACK_SIGNATURE)
{
    this->callback = callback;
}

uint8_t calculate_checksum(const char* pData, int len)
{
    uint8_t x = 0;
    for (int i=0; i<len; i++) {
        x = (x + pData[i]);// % 256;
    }
    return x;
}
_vallox_packet* CVallox::tryReadPacket()
{
    _vallox_packet* pPacket = (_vallox_packet*) &m_buffer[0];

    // read without timeout
    int n = mod.read(m_buffer, 1);
    if (n < 1)
        return NULL;
    
    // first byte (domain) MUST be VX_DOMAIN, check is it correct, if not ignore byte (shifting packet start pos)
    if (pPacket->domain != VX_DOMAIN) {
        DBG("reading domain, got garbage: %d", pPacket->domain);
        m_lastSerialActivity.start();
        return NULL;
    }

    // ok, firt byte is correct, continue reading (with timeout) rest of packet
    n = mod.readBytes(m_buffer+1, 5);
    if (n > 0) {
        m_lastSerialActivity.start();
    }
    if (n != 5) {
        DBG("failed to read whole packet, len: %d", n+1 );
        return NULL; // failed to read whole packet
    }

    // now whole packet is read, validate it
    uint8_t chk_calc = calculate_checksum((char*)m_buffer, 5);

    if (chk_calc != pPacket->checksum)
    {
        String log;
        log += "oops, checksum missmatch, packet:";
        log +="\n  domain: " + String(pPacket->domain, 16);
        log +="\n  sender: " + String( pPacket->sender, 16);
        log +="\n  receiver: " + String(pPacket->receiver, 16);
        log +="\n  param: " + String(pPacket->param, 16);
        log +="\n  data: " + String(pPacket->data, 16);
        log +="\n  checksum:" + String(pPacket->checksum, 16);
        log += "\n";
        
        DBG(log);
        /*DBG("oops, checksum missmatch");
        DBG("  domain: %d", packet->domain);
        DBG("  sender: %d", packet->sender);
        DBG("  receiver: %d", packet->receiver);
        DBG("  param: %d", packet->param);
        DBG("  data: %d", packet->data);
        DBG("  checksum: %d\n\n", packet->checksum);*/
        return NULL;
    }

    // pointer is local, but points to m_buffer start pos. safe.
    return pPacket;
}
void CVallox::loop()
{
    _vallox_packet* packet = tryReadPacket();
    if (packet) {
        //DBG("got packet ok");
        handlePacket(packet);
    }

    if (m_lastPing.elapsed() > 60000)
    {
        DBG("alive");
        m_lastPing.start();
    }

    if (m_lastQuery.elapsed() > 1000 && m_lastSerialActivity.elapsed() > 50)
    {
        // do query
        checkAndRefresh(VX_PARAM_LEDS);
        checkAndRefresh(VX_PARAM_FAN_SPEED);
        
        //checkAndRefresh(VX_PARAM_FAN_RELAYS);
        checkAndRefresh(VX_PARAM_IO_PORT_7);
        checkAndRefresh(VX_PARAM_IO_PORT_8);
        // etulämmityksen tilalippu,
        checkAndRefresh(VX_PARAM_FLAGS_5);
        // takkatoiminto
        checkAndRefresh(VX_PARAM_FLAGS_6);
        checkAndRefresh(VX_PARAM_TEMP_OUTDOOR);
        checkAndRefresh(VX_PARAM_TEMP_EXHAUST);
        checkAndRefresh(VX_PARAM_TEMP_INSIDE);
        checkAndRefresh(VX_PARAM_TEMP_INCOMING);
        checkAndRefresh(VX_PARAM_LAST_ERRORCODE);
        checkAndRefresh(VX_PARAM_TAKKA_COUNTER);
        checkAndRefresh(VX_PARAM_POST_HEATING);
        m_lastQuery.start();
    }
}

int CVallox::setFanSpeed(int speed)
{
    int retVal = 0;
    uint8_t fanSpeedData = getFanSpeedRaw(speed);
    if (fanSpeedData == 0)
    {
        DBG("setFanSpeed, invalid speed: %d", speed);
        retVal = -1;
    }
    else
        retVal = sendRequest(VX_PARAM_FAN_SPEED, fanSpeedData);

    /*
    fan_speed_val = getKeyByValue(VALLOX.FAN_SPEED, value)
    if fan_speed_val == None:
        logging.warning("set_fan_speed: Invalid value: " + str(value))
        return False
    else:
        logging.info("setting fan speed: " + str(value))
        return self.send_request(VALLOX.VX_PARAM_FAN_SPEED, fan_speed_val)
        */
    return retVal;
}
int CVallox::activateTakkakytkin()
{
    int retVal = 0;
    // read param VX_PARAM_FLAGS_6 and change 5th bit to true
    // bitti 5 = takkakytkimen aktivointi (write)
    std::map<uint8_t, uint8_t>::const_iterator it = m_cacheData.find(VX_PARAM_FLAGS_6);
    uint8_t cacheval = 0;
    if (it != m_cacheData.end())
    {
        // data found from cache
        cacheval = it->second;
    }
    else
    {
        // Query
        retVal = queryParam(VX_PARAM_FLAGS_6, cacheval);
        if (retVal < 0)
        {
            DBG("activateTakkakytkin() Failed to query VX_PARAM_FLAGS_6");
            //return -1;
        }
    }

    if (retVal == 0)
    {
        uint8_t newVal = set_bit(cacheval, 5);
        retVal = sendRequest(VX_PARAM_FLAGS_6, newVal);

        m_cacheTime.erase(VX_PARAM_TAKKA_COUNTER);
        m_cacheData.erase(VX_PARAM_TAKKA_COUNTER);
    }

    return retVal;
}
int CVallox::setPostHeatingTemp(int temperature)
{
    if (temperature < 10)
    {
        DBG("setPostHeatingTemp failed, too low: %d", temperature);
        return -1;
    }
    else if (temperature > 20)
    {
        DBG("setPostHeatingTemp failed, too high: %d", temperature);
        return -1;
    }

    uint8_t ntcVal = getNtcValue(temperature);
    if (ntcVal == 0)
    {
        DBG("setPostHeatingTemp failed, getNtcValue failed!");
        return -1;
    }

    int retVal = sendRequest(VX_PARAM_POST_HEATING, ntcVal);
    return retVal;
}

bool isParamTemperature(uint8_t param) {

    return ((param >= VX_PARAM_TEMP_OUTDOOR && param <= VX_PARAM_TEMP_INCOMING)
        || param == VX_PARAM_POST_HEATING);
}

char* CVallox::toJson() const
{
    StaticJsonDocument<800> doc;
    JsonObject root = doc.to<JsonObject>();

    std::map<uint8_t, uint8_t>::const_iterator it = m_cacheData.begin();
    while (it != m_cacheData.end())
    {
        uint8_t param = it->first;
        uint8_t value = it->second;
        it++;

        if (is_param_known_bitfield(param))
        {
            //DBG("handle_response_data() handle_value_bits_data");
            //handle_value_bits_data(param, value);
            //DBG("handle_response_data() handle_value_bits_data done");
            //return;

            for (int i=0; i<8; i++)
            {
                //if (!has_param_bit_changed(param, data, i))
                //    continue;

                const char* paramName = getParamBitName(param, i);
                if (!paramName)
                    continue;

                bool bitValue = get_bit(value, i);
                if (param == VX_PARAM_FLAGS_5 && i == 7) // etulammitys is inverted
                    bitValue = !bitValue;
                /*if (bitValue)
                    send_mqtt(topic, "On");
                else
                    send_mqtt(topic, "Off");*/
                
                root[paramName] = bitValue;
            }
            
            continue;
        }
        const char* paramName = getParamName(param);
        if (!paramName)
            continue;
        
        if (param == VX_PARAM_FAN_SPEED) // fan speed
        {
            //logging.debug("fan speed: " + str(FAN_SPEED.get(data)))
            //logging.debug ("topic: " + topic_for_param(param) + " " + str(FAN_SPEED.get(data)));
            //send_mqtt(topic, str(.FAN_SPEED.get(data)));
            //send_mqtt(topic, getFanSpeed(value));
            //m_lastPublishTime[param] = now;
            root[paramName] = getFanSpeed(value);
        }
        else if (isParamTemperature(param)) {

            int temperature = getTemp(value);
            
            if (temperature != INT_MIN) {
                root[paramName] = temperature;
            }
            
        }
        else {
            root[paramName] = value;
        }
    }

    char* jsonBuffer = (char*)malloc(800);
    serializeJson(root, jsonBuffer, 800);
    return jsonBuffer;
}


int CVallox::handlePacket(_vallox_packet* packet)
{

    //DBG("domain: %d", packet->domain);
    //DBG("sender: %d", packet->sender);
    //DBG("receiver: %d", packet->receiver);
    //DBG("param: %d", packet->param);
    //DBG("data: %d", packet->data);
    //DBG("checksum: %d", packet->checksum);

    if (packet->domain != VX_DOMAIN) { // domain must be 0x01
        String log;
        log += "invalid domain, packet:";
        log +="\n  domain: " + String(packet->domain, 16);
        log +="\n  sender: " + String( packet->sender, 16);
        log +="\n  receiver: " + String(packet->receiver, 16);
        log +="\n  param: " + String(packet->param, 16);
        log +="\n  data: " + String(packet->data, 16);
        log +="\n  checksum:" + String(packet->checksum, 16);

        /*DBG("invalid domain");
        DBG("  domain: %d", packet->domain);
        DBG("  sender: %d", packet->sender);
        DBG("  receiver: %d", packet->receiver);
        DBG("  param: %d", packet->param);
        DBG("  data: %d", packet->data);
        DBG("  checksum: %d", packet->checksum);*/
        uint8_t chk_calc = calculate_checksum((char*)packet, 5);
        if (chk_calc != packet->checksum) {
            log += "  (invalid checksum)";
        }
        log += "\n\n";
        
        DBG(log);
        
        return 0;
    }

    // VX_DOMAIN, CLIENT_ID, VX_MAINBOARD, VX_REQ_QUERY
    /*#ifdef _TRACE_COMMANDS
    if (packet->receiver == VX_MAINBOARD && packet->param != VX_REQ_QUERY)
    {
        param = xb[3]
        data = xb[4]
        logging.debug("traced cmd: " + hex(param) + ", data: " + hex(data))
        return
    }
    #endif*/

    

    if (packet->receiver != VX_ALL_CLIENTS && packet->receiver != KClient_id) // filter common or for us)
        return 0;

    //DBG("handle param: 0x%02X, data: 0x%02X", packet->param, packet->data);

    onParamReceived(packet->param, packet->data);

    m_cacheTime[packet->param] = millis();
    m_cacheData[packet->param] = packet->data;

    return 0;
}

void CVallox::onParamReceived(uint8_t param, uint8_t value)
{
    bool dataChanged(true);
    std::map<uint8_t, uint8_t>::const_iterator it = m_cacheData.find(param);
    uint8_t cacheData = 0;
    if (it != m_cacheData.end())
    {
        // data found from cache
        cacheData = it->second;
        dataChanged = (value != cacheData);
    }


    if (dataChanged)
    {
        // data changed
        DBG("param value changed: param: 0x%02X, data: 0x%02X", param, value);
        if (is_param_known_bitfield(param)) {
            DBG("known bitfield");
        }
        else {
            const char* pName = getParamName(param);
            if (pName) {
                DBG(pName);
            }
            else {
                DBG("unknown param");
            }
        }
        if (callback) {
            callback();
        }
    }


    if (param == VX_PARAM_SENDING_ALLOWED)
    {
        DBG("VX_PARAM_SENDING_ALLOWED");
        return;
    }
    else if (param == VX_PARAM_SENDING_DISALLOED)
    {
        DBG("VX_PARAM_SENDING_DISALLOED");
        return;
    }

    if (is_param_known_bitfield(param))
    {
        //DBG("handle_response_data() handle_value_bits_data");
        handle_value_bits_data(param, value);
        //DBG("handle_response_data() handle_value_bits_data done");
    }
}

void CVallox::handle_value_bits_data(uint8_t param, uint8_t data)
{
    for (int i=0; i<8; i++)
    {
        if (!has_param_bit_changed(param, data, i))
            continue;

        const char* paramName = getParamBitName(param, i);
        if (!paramName) {
            continue;
        }
/*
        bool bitValue = get_bit(data, i);
        if (param == VX_PARAM_FLAGS_5 && i == 7) // etulammitys is inverted
            bitValue = !bitValue;
        if (bitValue)
            send_mqtt(topic, "On");
        else
            send_mqtt(topic, "Off");
        */
    }
    //const char* topic = getTopicForParamBit(param, bit);
}

bool CVallox::has_param_bit_changed(uint8_t param, uint8_t data, uint8_t bitIndex)
{
    std::map<uint8_t, uint8_t>::const_iterator it = m_cacheData.find(param);
    if (it == m_cacheData.end()) // not found, changed
        return true;

    bool changed = (get_bit(it->second, bitIndex) != get_bit(data, bitIndex));
    return changed;
    
}

bool CVallox::hasRecentData(uint8_t param)
{
    long int now = millis();
    std::map<uint8_t, long int>::const_iterator it = m_cacheTime.find(param);
    if (it != m_cacheTime.end())
    {
        // time found from cache
        if ((now - it->second) < 20000)
            return true; // recently updated
    }
    return false;
}
bool isReplyForParam(const _vallox_packet* pPacket, uint8_t param)
{
    if (pPacket->domain == VX_DOMAIN
        && pPacket->sender == VX_MAINBOARD
        && pPacket->receiver == KClient_id
        && pPacket->param == param)
        {
            // ok, if checksum is ok
            return pPacket->checksum == calculate_checksum((char*)pPacket, 5);
        }
    else
        return false;
}
int CVallox::wait_for_idle()
{
    char buf[1];
    do
    {
        int n = mod.read(buf, sizeof buf);
        if (n > 0)
            m_lastSerialActivity.start();
        else {
            // wait 1 ms
            delayMicroseconds(1000);
        }
    } while (m_lastSerialActivity.elapsed() < 5); // loop until quiet for 5 ms

    return 0;
}
size_t CVallox::write(const char* buffer, size_t size) {
    digitalWrite(RE,HIGH);
    digitalWrite(DE,HIGH);
    size_t writed = mod.write(buffer, size);
    digitalWrite(RE,LOW);
    digitalWrite(DE,LOW);
    return writed;
}
int CVallox::checkAndRefresh(uint8_t param)
{
    if (hasRecentData(param))
        return 0;

    uint8_t value(0);
    int retVal = queryParam(param, value);
    if (retVal == 0)
    {
        onParamReceived(param, value);
        m_cacheTime[param] = millis();
        m_cacheData[param] = value;
    }
    return retVal;
}
int CVallox::queryParam(uint8_t param, uint8_t& value)
{
    //DBG("queryParam param: 0x%02X", param);

    char writeBuffer[6];
    create_querybuf(&writeBuffer[0], param);

    int failurecount = 0;

    wait_for_idle();

    CElapseTimer requestTimer;
    if (write(writeBuffer, sizeof writeBuffer) < 0)
        return -1;

    m_lastSerialActivity.start();
    requestTimer.start();
    while (1)
    {
        if (requestTimer.elapsed() > 10 && m_lastSerialActivity.elapsed() > 3) // Failed if took more than 20 ms, and idle
        {
            failurecount++;
            if (failurecount < 10)
            {
                // try again
                if (write(writeBuffer, sizeof writeBuffer) < 0)
                    break;
                requestTimer.start();
                m_lastSerialActivity.start();
            }
            else
            {
                DBG("queryParam() failed too many times, abort");
                break;
            }
        }

        _vallox_packet* pPacket = tryReadPacket();
        if (pPacket) {
            // got packet
            if (isReplyForParam(pPacket, param))
            {
                if (failurecount > 0) {
                    DBG("got reply after %d failures", failurecount);
                }
                // got reply
                //long timetook = requestTimer.elapsedUSec();
                //DBG("got reply, timetook %ld us", timetook);
                //DBG("got reply");
                value = pPacket->data;
                return 0;
            }
        }
        
        // no data or invalid packet, wait 500 us
        //delayMicroseconds(500);
        optimistic_yield(1000UL);
    }

    // failed
    DBG("queryparam failed");
    return -1;
}
int CVallox::sendRequest(uint8_t param, uint8_t data)
{
    DBG("sendRequest() param: 0x%02X, data: 0x%02X", param, data);

    char writeBuffer[6];
    create_requestbuf(&writeBuffer[0], param, data);

    // TODO: send request to serial

    /*DBG("sendRequest data:");
    for (int i=0; i<6; i++)
    {
        DBG("  0x%02X", (uint8_t)writeBuffer[i]);
    }*/

    wait_for_idle();

    int writed = write(writeBuffer, sizeof writeBuffer);
    if (writed < 0)
        return -1;

    CElapseTimer requestTimer;
    requestTimer.start();

    int failurecount = 0;
    int retVal = -1;
    char readBuffer[1];
    do
    {
        if (requestTimer.elapsed() > 20 && m_lastSerialActivity.elapsed() > 3) // Failed if took more than 20 ms, and idle
        {
            failurecount++;
            if (failurecount < 2)
            {
                int writed = write(writeBuffer, sizeof writeBuffer);
                DBG("retry, write ret: %d", writed);
                requestTimer.start();
                m_lastSerialActivity.start();
            }
            else
            {
                DBG("sendRequest() failed too many times, abort");
                break;
            }
        }
        int n = mod.read(readBuffer, sizeof readBuffer);  // read up to x characters if ready to read
        if (n > 0)
        {
            m_lastSerialActivity.start();
            // we expect first byte == request checksum
            if (readBuffer[0] == writeBuffer[5])
            {
                DBG("got expected");
                retVal = 0;
                break;
            }
            else {
                DBG("got unexpected: %d", readBuffer[0]);
            }
        }
        else
        {
            // no data, wait 500 us
            delayMicroseconds(500);
        }

    } while(1);
    DBG("sendRequest() done, retVal: %d", retVal);

    // Invalidate cache
    m_cacheTime.erase(param);
    m_cacheData.erase(param);

    return retVal;
}

void CVallox::create_querybuf(void* pBuf, uint8_t param)
{
    _vallox_packet* pPacket = (_vallox_packet*)pBuf;
    pPacket->domain = VX_DOMAIN;
    pPacket->sender = KClient_id;
    pPacket->receiver = VX_MAINBOARD;
    pPacket->param = VX_REQ_QUERY;
    pPacket->data = param;
    pPacket->checksum = calculate_checksum((char*)pPacket, 5);
}

void CVallox::create_requestbuf(void* pBuf, uint8_t param, uint8_t data)
{
    _vallox_packet* pPacket = (_vallox_packet*)pBuf;
    pPacket->domain = VX_DOMAIN;
    pPacket->sender = KClient_id;
    pPacket->receiver = VX_MAINBOARD;
    pPacket->param = param;
    pPacket->data = data;
    pPacket->checksum = calculate_checksum((char*)pPacket, 5);
}

void CVallox::dotests()
{

    /*
    char buf[6];
    _vallox_packet* packet = (_vallox_packet*) &buf[0];

    packet->domain = 0x01;
    packet->sender = VX_MAINBOARD;
    packet->receiver = VX_ALL_CLIENTS;
    packet->param = VX_PARAM_TEMP_OUTDOOR;
    packet->data = 0x52;
    packet->checksum = calculate_checksum((char*)packet, 5);

    handlePacket(packet);

    packet->data = 0x53;
    packet->checksum = calculate_checksum((char*)packet, 5);
    handlePacket(packet);
*/
/*
    CElapseTimer el;

    long int m = millis();
    el.start();
    //usleep(15276);
    sleep(20);
    long long elapsed = el.elapsedUSec();
    long int m_elaps = millis() - m;
    DBG("m_elaps  : %ld", m_elaps);
    DBG("u elapsed: %ld", elapsed);
*/

}
