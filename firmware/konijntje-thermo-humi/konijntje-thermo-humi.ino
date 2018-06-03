#include "Arduino.h"
#include "stm32i2c.h"
#include "HTU21D.h"

stm32I2c i2c;
HTU21D tSensor(&i2c);
unsigned long ulTimer=0;

void setup()
{
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Ready...");
    if(!tSensor.init(HTU21D::RH_12b_TEMP_14b))
    {
        Serial.println("Sensor not ready");
        return;
    }
}

void loop()
{
    float fTemp;
    if(millis()>ulTimer+1000)
    {
        ulTimer=millis();
        if(tSensor.readTemperature(fTemp))
        {
            Serial.println(fTemp);
        }
    }
}
