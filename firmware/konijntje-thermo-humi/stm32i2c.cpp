#include "stm32i2c.h"
#include <Wire.h>
#ifdef ARDUINO_STM_NUCLEO_F103RB
//SCL = SCL/D15
//SDA = SDA/D14
HardWire HWire(1, I2C_REMAP);// | I2C_BUS_RESET); // I2c1
#else
#define HWire Wire
#endif

stm32I2c::stm32I2c()
{

}

bool stm32I2c::begin(byte address)
{
    HWire.begin();
    _address=address;
    return isPresent();
}

bool stm32I2c::isPresent()
{
    HWire.beginTransmission(_address);
    return (HWire.endTransmission() == 0);
}

bool stm32I2c::writeByte(byte data)
{
    HWire.beginTransmission(_address);
    HWire.write(data);
    return HWire.endTransmission()==0;
}

bool stm32I2c::writeBytes(byte *data, byte length)
{
    HWire.beginTransmission(_address);
    HWire.write(data, length);
    return HWire.endTransmission()==0;
}

bool stm32I2c::readBytes(byte* data, byte length)
{
    byte nrOfBytesRead=0;
    HWire.requestFrom(_address, length);
    while(nrOfBytesRead<length)
    {
        if(!HWire.available())
        {
            break;
        }
        data[nrOfBytesRead++] = HWire.read();
    }
    return nrOfBytesRead==length;
}

