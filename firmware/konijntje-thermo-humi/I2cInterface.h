#pragma once

#include "Arduino.h"

class I2cInterface
{
public:
    virtual bool begin(byte address);
    virtual bool isPresent();
    virtual bool writeByte(byte data);
    virtual bool writeBytes(byte* data, byte length);
    virtual bool readBytes(byte* data, byte length);
protected:
    byte _address;
};
