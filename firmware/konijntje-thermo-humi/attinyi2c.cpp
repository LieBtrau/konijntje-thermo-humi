#include "attinyi2c.h"
#include "Wire.h"

AttinyI2c::AttinyI2c()
{

}

bool AttinyI2c::begin(byte address)
{
    Wire.begin();
    _address=address;
    return isPresent();
}

bool AttinyI2c::isPresent()
{
    Wire.beginTransmission(_address);
    return (Wire.endTransmission() == 0);
}

bool AttinyI2c::writeByte(byte data)
{
    Wire.beginTransmission(_address);
    Wire.write(data);
    return Wire.endTransmission()==0;
}

bool AttinyI2c::writeBytes(byte *data, byte length)
{
    Wire.beginTransmission(_address);
    Wire.write(data, length);
    return Wire.endTransmission()==0;
}

bool AttinyI2c::readBytes(byte* data, byte length)
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
