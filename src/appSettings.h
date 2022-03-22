#ifndef APPSETTINGS_H
#define APPSETTINGS_H

#include "Arduino.h"


class AppSettings {
    public:
        AppSettings();
        ~AppSettings();

        void clear();
        void print() const;
        bool isValid() const;

    public:
        char* m_mqtt_server;
        int m_mqtt_port;
        char* m_mqtt_user;
        char* m_mqtt_passw;
};

#endif