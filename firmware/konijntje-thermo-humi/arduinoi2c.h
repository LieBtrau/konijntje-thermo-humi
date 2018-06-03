#pragma once

#include "I2cInterface.h"

class ArduinoI2C : public I2cInterface
{
public:
    ArduinoI2C();
    bool begin(byte address);
    bool isPresent();
    bool writeByte(byte data);
    bool writeBytes(byte* data, byte length);
    bool readBytes(byte* data, byte length);
};

