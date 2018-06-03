#pragma once
#include "I2cInterface.h"

class stm32I2c : public I2cInterface
{
public:
    stm32I2c();
    bool begin(byte address);
    bool isPresent();
    bool writeByte(byte data);
    bool writeBytes(byte* data, byte length);
    bool readBytes(byte* data, byte length);
};
