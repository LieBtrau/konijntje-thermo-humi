#include "Arduino.h"
#include "HTU21D.h"

#include "Chaplex.h"

byte ctrlpins[] = {1,7,8,9};    //Arduino pins controlling charlieplexed leds

Chaplex myCharlie(ctrlpins, sizeof(ctrlpins));     //control instance

charlieLed myLeds[11]  = {
    { 1 , 3 },  //18 - 25
    { 3 , 0 },  //19 - 30
    { 0 , 3 },  //20 - 35
    { 3 , 2 },  //21 - 40
    { 2 , 3 },  //22 - 45
    { 2 , 0 },  //23 - 50
    { 0 , 2 },  //24 - 55
    { 2 , 1 },  //25 - 60
    { 1 , 2 },  //26 - 65
    { 0 , 1 },  //27 - 70
    { 1 , 0 }   //28 - 75
};


#ifdef ARDUINO_STM_NUCLEO_F103RB
#include "stm32i2c.h"
stm32I2c i2c;
HardwareSerial* SerialPort=&Serial;
#elif defined(ARDUINO_AVR_ATTINYX4)
#include <SoftwareSerial.h>
SoftwareSerial mySerial(0, 3); // RX, TX
SoftwareSerial* SerialPort=&mySerial;
#include "arduinoi2c.h"
ArduinoI2C i2c;
#endif
HTU21D tSensor(&i2c);
unsigned long ulTimer=0;
const byte SENSORPWR=2;
byte led=0;

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
    myCharlie.setSingleLed(myLeds[5],ON);
}

void loop()
{
    int iTemp;
    byte yHumi;
    bool bHeater=false;
    if(millis()>ulTimer+1000)
    {
        ulTimer=millis();
        if(tSensor.readTemperature(iTemp) && tSensor.readHumidity(yHumi, bHeater))
        {
#ifdef ARDUINO_STM_NUCLEO_F103RB
            char s[80];
            sprintf(s, "Temperature : %d°C \tHumidity: %d%%", iTemp, yHumi);
            SerialPort->println(s);
#elif defined(ARDUINO_AVR_ATTINYX4)
            SerialPort->println(iTemp);
            SerialPort->println(yHumi);
#endif
        }
//        myCharlie.ledWrite(myLeds[led],OFF);
        led = led+1 >= (sizeof(myLeds)>>1) ? 0 : led+1;
//        myCharlie.ledWrite(myLeds[led], ON);
        myCharlie.setSingleLed(myLeds[led], ON);
    }
    //myCharlie.update();
}
