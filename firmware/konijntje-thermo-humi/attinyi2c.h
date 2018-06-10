#pragma once
#include "I2cInterface.h"

class AttinyI2c : public I2cInterface
{
public:
    AttinyI2c();
    bool begin(byte address);
    bool isPresent();
    bool writeByte(byte data);
    bool writeBytes(byte* data, byte length);
    bool readBytes(byte* data, byte length);
};
