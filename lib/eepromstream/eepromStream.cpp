#include <eepromStream.h>
#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include <stddef.h>
    #include <cstring>
#endif

/*template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
   const byte* p = (const byte*)(const void*)&value;
   int i;
   for (i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
   return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
   byte* p = (byte*)(void*)&value;
   int i;
   for (i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return i;
}*/

template <class T> int EEPROM_writeAnything(EepromStream* pStream, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
        pStream->writeInt8(*p++);
    return i;
}

template <class T> int EEPROM_readAnything(EepromStream* pStream, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = pStream->readInt8();
    return i;
}

EepromStream::EepromStream() : m_pBegin(NULL)
{

}
EepromStream::~EepromStream()
{

}

void EepromStream::setUnderlyingData(void* pData, int maxLen)
{
    m_pBegin = (uint8_t*)pData;
    m_pPos = m_pBegin;
}
void EepromStream::seek(int pos)
{

}

int8_t EepromStream::readInt8()
{
    int8_t val = *m_pPos;
    m_pPos++;
    return val;
}
int16_t EepromStream::readInt16()
{
    int16_t val;
    memcpy(&val, m_pPos, sizeof(val));
    m_pPos += sizeof(val);
    return val;
}
int32_t EepromStream::readInt32()
{
    int32_t val;
    memcpy(&val, m_pPos, sizeof(val));
    m_pPos += sizeof(val);
    return val;
}
int64_t EepromStream::readInt64()
{
    int64_t val;
    memcpy(&val, m_pPos, sizeof(val));
    m_pPos += sizeof(val);
    return val;
}
float EepromStream::readFloat()
{
    float f;
    EEPROM_readAnything(this, f);
    return f;
}

void EepromStream::writeInt8(int8_t val)
{
    *m_pPos = val;
    m_pPos++;
}
void EepromStream::writeInt16(int16_t val)
{
    uint8_t* pVal = (uint8_t*) &val;
    memcpy(m_pPos, pVal, sizeof(val));
    m_pPos += sizeof(val);
}
void EepromStream::writeInt32(int32_t val)
{
    uint8_t* pVal = (uint8_t*) &val;
    memcpy(m_pPos, pVal, sizeof(val));
    m_pPos += sizeof(val);
}
void EepromStream::writeInt64(int64_t val)
{
    uint8_t* pVal = (uint8_t*) &val;
    memcpy(m_pPos, pVal, sizeof(val));
    m_pPos += sizeof(val);
}
void EepromStream::writeFloat(float val)
{
    EEPROM_writeAnything(this, val);
}

// reads until null character found
const char* EepromStream::readString()
{
    const char* pStr = (const char*)m_pPos;
    // seek over the null
    m_pPos += (strlen(pStr)+1);
    return pStr;
}
char* EepromStream::readStringDup()
{
    const char* pStr = readString();
    char* pCopy = NULL;
    if (pStr) 
        pCopy = strdup(pStr);
    return pCopy;
}
char* EepromStream::readStringTo(char* pTarget)
{
    return strcpy(pTarget, readString());
}

void EepromStream::writeString(const char* pTxt)
{
    if (pTxt) {
        size_t dataLen = strlen(pTxt)+1;
        // write string and null
        memcpy(m_pPos, pTxt, dataLen);
        m_pPos += dataLen;
    }
    else {
        writeInt8(0); // write null
    }
}