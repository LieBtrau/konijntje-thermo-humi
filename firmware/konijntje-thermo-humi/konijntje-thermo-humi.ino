#include "Arduino.h"
#include "HTU21D.h"
#include <Bounce2.h>
#include "Chaplex.h"
//#include <SoftwareSerial.h>
#include "arduinoi2c.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

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
//SoftwareSerial mySerial(0, 3); // RX, TX
//SoftwareSerial* SerialPort=&mySerial;
ArduinoI2C i2c;
HTU21D tSensor(&i2c);
unsigned long ulStateTimer=0;
const byte BUTTON_PIN=0;
const byte SENSORPWR_PIN=2;
const byte HUMIDITY_PIN=10;

//http://johnsantic.com/comp/state.html
enum states { SLEEPING, SHOWING_TEMP, SHOWING_HUMIDITY, MAX_STATES } current_state;
enum events { TIMER_RUNOUT, BUTTON_PUSHED, MAX_EVENTS } new_event;
enum events get_new_event (void);

void readHumidity(void);
void doNothing(void);
void goToSleep(void);
void readTemperature(void);
void (*const state_table [MAX_STATES][MAX_EVENTS]) (void) =
{
{ doNothing, readTemperature },     //sleeping
{ goToSleep, readHumidity },        //showingTemp
{ goToSleep, goToSleep }            //showingHumidity
        };

void setup()
{
    disable_wdt();
    OSCCAL-=3;                          //user RC calibration
    pinMode(HUMIDITY_PIN, OUTPUT);
    pinMode(SENSORPWR_PIN, OUTPUT);
    pinMode(BUTTON_PIN,INPUT_PULLUP);
    debouncer.attach(BUTTON_PIN);
    debouncer.interval(5);
    int supplyVoltage=getVCC();
    if(supplyVoltage<2500)
    {
        digitalWrite(HUMIDITY_PIN, HIGH);
        myCharlie.setSingleLed(myLeds[11], ON);
        delay(1000);
        goToSleep();
    }
    readTemperature();
}

void loop(void)
{
    new_event = get_new_event ();

    if (((new_event >= 0) && (new_event < MAX_EVENTS)) && ((current_state >= 0) && (current_state < MAX_STATES)))
    {
        state_table [current_state][new_event] ();
    }
}

void readTemperature(void)
{
    int iTemp;
    digitalWrite(SENSORPWR_PIN, HIGH);
    delay(500);
    if(!tSensor.init(HTU21D::RH_12b_TEMP_14b))
    {
        return;
    }
    if(tSensor.readTemperature(iTemp))
    {
        //Round off temperature and convert to °C
        iTemp=(iTemp+50)/100;
        //SerialPort->println(iTemp);
        char cLedIndex=iTemp-18;
        if(cLedIndex<0)
        {
            cLedIndex=0;
        }
        if(cLedIndex>10)
        {
            cLedIndex=10;
        }
        myCharlie.allClear();
        myCharlie.setLedState(myLeds[11], ON);
        myCharlie.setLedState(myLeds[cLedIndex], ON);
        ulStateTimer=millis();
        current_state = SHOWING_TEMP;
    }
}

void readHumidity(void)
{
    byte yHumi;
    bool bHeater=false;
    if(tSensor.readHumidity(yHumi,bHeater))
    {
        //SerialPort->println(yHumi);
        int iLedIndex = ((yHumi*10)+25)/50-5;
        if(iLedIndex<0)
        {
            iLedIndex=0;
        }
        if(iLedIndex>10)
        {
            iLedIndex=10;
        }
        myCharlie.allClear();
        myCharlie.setLedState(myLeds[iLedIndex], ON);
        digitalWrite(HUMIDITY_PIN, HIGH);
        ulStateTimer=millis();
        current_state = SHOWING_HUMIDITY;
    }
}

void goToSleep(void)
{
    //SerialPort->println("Zzz...");
    myCharlie.allClear();
    myCharlie.showLedState();
    digitalWrite(HUMIDITY_PIN, LOW);
    digitalWrite(SENSORPWR_PIN, LOW);
    current_state = SLEEPING;
    sleepMcu();
}

void doNothing(void)
{
    ulStateTimer=millis();
}

enum events get_new_event (void)
{
    while(true)
    {
        myCharlie.showLedState();
        debouncer.update();
        if (debouncer.fell())
        {
            return BUTTON_PUSHED;
        }
        if(millis()>ulStateTimer+5000)
        {
            return TIMER_RUNOUT;
        }
    }
}

void sleepMcu(){
    bitSet(PCMSK0, PCINT0);
    bitSet(GIFR, PCIF0);
    bitSet(GIMSK, PCIE0);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    bitClear(ADCSRA, ADEN); //disable ADC
    bitSet(ACSR, ACD); //disable Analog comparator
    DIDR0=0xFF;//disable digital input buffer for analog pins
    PRR=0x0F;
    sleep_enable();
    sei();
    sleep_cpu();
}

ISR (PCINT0_vect)
{
    //After ISR PCINT0_vect executed, there will be no reset, but PC will jump to the line following here:
    sleep_disable();
    //Force reset by use of watchdog
    wdt_enable(WDTO_15MS);
    while(1);
}

//Once enabled, the wdt is not disabled by a software reset.
void disable_wdt()
{
    wdt_reset();
    /* Clear WDRF in MCUSR */
    MCUSR = 0x00;
    /* Write logical one to WDCE and WDE */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
}

//https://github.com/cano64/ArduinoSystemStatus/blob/master/SystemStatus.cpp
int getVCC() {
    //reads internal 1V1 reference against VCC
    ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA, ADSC));
    uint8_t low = ADCL;
    unsigned int val = (ADCH << 8) | low;
    //discard previous result
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA, ADSC));
    low = ADCL;
    val = (ADCH << 8) | low;

    return ((long)1024 * 1100) / val;
}

