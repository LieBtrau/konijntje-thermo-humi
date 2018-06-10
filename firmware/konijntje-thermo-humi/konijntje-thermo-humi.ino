#include "Arduino.h"
#include "HTU21D.h"

#ifdef ARDUINO_STM_NUCLEO_F103RB
#include "stm32i2c.h"
stm32I2c i2c;
HardwareSerial* SerialPort=&Serial;
#elif defined(ARDUINO_AVR_ATTINYX4)
#include <SoftwareSerial.h>
SoftwareSerial mySerial(0, 3); // RX, TX
SoftwareSerial* SerialPort=&mySerial;
#include "attinyi2c.h"
AttinyI2c i2c;
#endif
HTU21D tSensor(&i2c);
unsigned long ulTimer=0;
const byte SENSORPWR=2;

void setup()
{
    SerialPort->begin(4800);
#ifdef ARDUINO_STM_NUCLEO_F103RB
    while (!Serial);
#elif defined(ARDUINO_AVR_ATTINYX4)
    OSCCAL-=3;                          //user RC calibration
    pinMode(SENSORPWR, OUTPUT);
    digitalWrite(SENSORPWR, HIGH);
#endif
    SerialPort->println("Ready...");
    if(!tSensor.init(HTU21D::RH_12b_TEMP_14b))
    {
        SerialPort->println("Sensor not ready");
        return;
    }
    SerialPort->println("Sensor ok");
}

void loop()
{
    float fTemp, fHumi;
    bool bHeater=false;
    if(millis()>ulTimer+1000)
    {
        ulTimer=millis();
        if(tSensor.readTemperature(fTemp) && tSensor.readHumidity(fHumi, bHeater))
        {
#ifdef ARDUINO_STM_NUCLEO_F103RB
            char s[80];
            sprintf(s, "Temperature : %2.1f°C \tHumidity: %2.2f%%", fTemp, fHumi);
            SerialPort->println(s);
#elif defined(ARDUINO_AVR_ATTINYX4)
            //no sprintf because it takes up too much space
            SerialPort->println(fTemp);
            SerialPort->println(fHumi);
#endif
        }
    }
}
