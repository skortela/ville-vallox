
#ifndef _VALLOX_H_
#define _VALLOX_H_

// pin configs
#define RE D5
#define RO D2
#define DE D6
#define DI D7

#include <map>
#include <sys/types.h>
#include <stdint.h>

#include <SoftwareSerial.h>
//#include <MQTTClient.h>
//#include <time.h>

#include "vallox_constants.h"

class PubSubClient;

class CElapseTimer
{
public:
    CElapseTimer()
    {
        m_millis = 0;
    };
    ~CElapseTimer(){};

    void start()
    {
        //clock_gettime(CLOCK_MONOTONIC, &m_ts);
        //long int ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
        m_millis = millis();
    }

    // returns elapsed microseconds since last call of start
    /*unsigned long int elapsedUSec()
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        unsigned long int us = (now.tv_sec - m_ts.tv_sec) * 1000000 + (now.tv_nsec - m_ts.tv_nsec) / 1000;
        return us;
    }*/
    // returns elapsed milliseconds since last call of start
    unsigned long elapsed()
    {
        return millis() - m_millis;
    }

private:
    unsigned long m_millis;
};

#if defined(ESP8266) || defined(ESP32)
#include <functional>
#define VALLOX_CALLBACK_SIGNATURE std::function<void()> callback
#else
#define VALLOX_CALLBACK_SIGNATURE void (*callback)()
#endif

class CVallox
{
public:
    CVallox();
    ~CVallox();

    void begin(int8_t rxPin, int8_t txPin);

    void setCallback(VALLOX_CALLBACK_SIGNATURE);

    void loop();

    // Set fan speed, (1 to 8)
    int setFanSpeed(int speed);
    int activateTakkakytkin();
    int setPostHeatingTemp(int temperature);

    char* toJson() const;

    void dotests();
private:
    // reads packet, or returns NULL 
    _vallox_packet* tryReadPacket();
    int handlePacket(_vallox_packet* packet);
    void onParamReceived(uint8_t param, uint8_t value);
    int checkAndRefresh(uint8_t param);
    int queryParam(uint8_t param, uint8_t& value);
    int sendRequest(uint8_t param, uint8_t data);

    void handle_value_bits_data(uint8_t param, uint8_t data);
    bool has_param_bit_changed(uint8_t param, uint8_t data, uint8_t bitIndex);
    bool hasRecentData(uint8_t param);

    void create_querybuf(void* pBuf, uint8_t param);
    // Formulates requestpacket from param and data
    // pBuf buffer, size must be at least 6 bytes
    void create_requestbuf(void* pBuf, uint8_t param, uint8_t data);
    int wait_for_idle();

    size_t write(const char* buffer, size_t size);

private:

    SoftwareSerial mod;
    VALLOX_CALLBACK_SIGNATURE;
    CElapseTimer m_lastQuery;
    CElapseTimer m_lastSerialActivity;
    CElapseTimer m_lastPing;
    std::map<uint8_t, long int> m_cacheTime;
    std::map<uint8_t, uint8_t> m_cacheData;

    char m_buffer[6];
};

#endif // _VALLOX_H_
