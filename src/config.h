#ifndef CONFIG_H
#define CONFIG_H

#include <inttypes.h>


#define KDeviceHostname "ville-vallox"

const char* const KMqttPrefix = "vallox/";
const char* const KMqttStateTopic = "vallox/state"; // for status updates
const char* const KMqttAvailability = "vallox/availability";
const char* const KMqttTopicCommandListen = "vallox/cmnd/#";
const char* const KOnline = "online";
const char* const KOffline = "offline";

const unsigned long KStatusUpdateInterval = 60000L;  // interval in ms.
const unsigned long KAvailabilityInterval = 300000L; // interval in ms.


#define KDefaultMqttPort 1883

// Magic number to check if eeprom config is valid
#define KConfigValidationMagic 0x09278263

#endif