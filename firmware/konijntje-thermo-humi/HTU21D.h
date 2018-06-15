/*
 HTU21D Humidity Sensor Library
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 22nd, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 Get humidity and temperature from the HTU21D sensor.

 Changes made by Christoph Tack, 2014:
 Also works for Silabs SI7020 & SI7021 sensors
 */

#pragma once

#include "Arduino.h"
#include "I2cInterface.h"

class HTU21D {

public:
    typedef enum
    {
        RH_12b_TEMP_14b,
        RH_8b_TEMP_12b,
        RH_10b_TEMP_13b,
        RH_11b_TEMP_11b
    } RESOLUTION;
    HTU21D(I2cInterface* pI2C);
    bool readHumidity(byte &humidity, bool &bHeaterOn);
    bool readTemperature(int &temp);
    bool init(RESOLUTION res);
    bool readSerialNumber(byte* sn);
    bool setHeater(bool bHeaterOn);
private:
    static const byte HTDU21D_ADDRESS=0x40;//bit[6..0]
    typedef enum
    {
        TRIGGER_HUMD_MEASURE_HOLD=0xE5,
        TRIGGER_HUMD_MEASURE_NOHOLD=0xF5,
        TRIGGER_TEMP_MEASURE_HOLD=0xE3,
        TRIGGER_TEMP_MEASURE_NOHOLD=0xF3,
        POST_RH_TEMP_READ=0xE0,
        SOFT_RESET=0xFE,
        READ_USER_REG= 0xE7,
        WRITE_USER_REG= 0xE6,
        SERIAL1_READA= 0xFA,
        SERIAL1_READB= 0x0F,
        SERIAL2_READA= 0xFC,
        SERIAL2_READB=0xC9
    } sensorCommands;
    typedef enum{
        RES1_BIT=7,
        HTRE_BIT=2,
        RES0_BIT=0
    } USERREG1;
    bool isCrcOk(byte* data, byte nrOfBytes, byte crcRead);
    byte updateCrc (byte* data, byte nbrOfBytes, byte crcIn);
    bool writeCmd(sensorCommands cmd);
    bool writeCmd(sensorCommands cmd, byte *data, byte datalen);
    bool readReg(byte* data, byte length);
    bool readReg(sensorCommands reg, byte* data, byte length);
    I2cInterface* _pI2c;
};

