#include <Arduino.h>
#include "debug.h"
#include <WebSerial.h>

void DBG(const char *format, ...)
{
    char buffer[65];
    va_list args;
    va_start (args, format);
    vsnprintf (buffer,65,format, args);
    va_end (args);

    WebSerial.println(buffer);
}
void DBG(const String& txt)
{
    WebSerial.println(txt);
}
