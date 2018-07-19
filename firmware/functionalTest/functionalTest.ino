#include "Arduino.h"
#include "HTU21D.h"
#include <Bounce2.h>
#include "Chaplex.h"
#include "arduinoi2c.h"

byte ctrlpins[] = {1,7,8,9};    //Arduino pins controlling charlieplexed leds
Chaplex myCharlie(ctrlpins, sizeof(ctrlpins));
Bounce debouncer = Bounce();
charlieLed myLeds[12]  =
{
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
    { 1 , 0 },  //28 - 75
    { 3 , 1 }   //Temperature LED
};
ArduinoI2C i2c;
HTU21D tSensor(&i2c);
unsigned long ulStateTimer=0;
const byte BUTTON_PIN=0;
const byte SENSORPWR_PIN=2;
const byte HUMIDITY_PIN=10;
byte curLed=0;

bool testTemperature(void);

void setup()
{
    OSCCAL-=3;                          //user RC calibration
    pinMode(HUMIDITY_PIN, OUTPUT);
    pinMode(SENSORPWR_PIN, OUTPUT);
    pinMode(BUTTON_PIN,INPUT_PULLUP);
    debouncer.attach(BUTTON_PIN);
    debouncer.interval(5);
    if(!testTemperature())
    {
        while(1);
    }
    myCharlie.allClear();
}

void loop(void)
{
    myCharlie.showLedState();
    debouncer.update();
    if(millis()>ulStateTimer+200)
    {
        ulStateTimer=millis();
        myCharlie.setLedState(myLeds[curLed], OFF);
        curLed=curLed<12 ? curLed+1 : 0;
        myCharlie.setLedState(myLeds[curLed], ON);
    }
    if (debouncer.fell())
    {
        digitalWrite(HUMIDITY_PIN, HIGH);
    }
    if(debouncer.read())
    {
        digitalWrite(HUMIDITY_PIN, LOW);
    }

}

bool testTemperature(void)
{
    int iTemp;
    digitalWrite(SENSORPWR_PIN, HIGH);
    delay(500);
    return tSensor.init(HTU21D::RH_12b_TEMP_14b);
}
