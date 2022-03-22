#ifndef EEPROMSTREAM_H
#define EEPROMSTREAM_H

#include <inttypes.h>


class EepromStream {

    public:
        EepromStream();
        ~EepromStream();

        void setUnderlyingData(void* pData, int maxLen);
        void seek(int pos);

        int8_t readInt8();
        int16_t readInt16();
        int32_t readInt32();
        int64_t readInt64();
        float readFloat();

        void writeInt8(int8_t val);
        void writeInt16(int16_t val);
        void writeInt32(int32_t val);
        void writeInt64(int64_t val);
        void writeFloat(float val);

        // reads until null character found
        const char* readString();
        // creates string copy
        char* readStringDup();
        char* readStringTo(char* pTarget);
        void writeString(const char* pTxt);
        
    
    private:
        uint8_t* m_pBegin;
        uint8_t* m_pPos;
};

#endif