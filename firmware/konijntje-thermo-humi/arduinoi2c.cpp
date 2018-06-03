#include "arduinoi2c.h"
#include <Wire.h>

ArduinoI2C::ArduinoI2C()
{

}

bool ArduinoI2C::begin(byte address)
{
    Wire.begin();
    _address=address;
    return isPresent();
}

bool ArduinoI2C::isPresent()
{
    Wire.beginTransmission(_address);
    return (Wire.endTransmission() == 0);
}

bool ArduinoI2C::writeByte(byte data)
{
    Wire.beginTransmission(_address);
    Wire.write(data);
    return Wire.endTransmission()==0;
}

bool ArduinoI2C::writeBytes(byte *data, byte length)
{
    Wire.beginTransmission(_address);
    Wire.write(data, length);
    return Wire.endTransmission()==0;
}

bool ArduinoI2C::readBytes(byte* data, byte length)
{
    byte nrOfBytesRead=0;
    Wire.requestFrom(_address, length);
    while(nrOfBytesRead<length)
    {
        if(!Wire.available())
        {
            break;
        }
        data[nrOfBytesRead++] = Wire.read();
    }
    return nrOfBytesRead==length;
}
