
 
void setup() {
    OSCCAL-=3;
    pinMode(8, OUTPUT);
    TCNT0 = 0;                  // Count up from 0
    bitClear(TCCR0A, WGM00);      // CTC mode
    bitSet(TCCR0A, WGM01);      // CTC mode
    bitClear(TCCR0B, WGM02);      // CTC mode
    bitSet(TCCR0A, COM0A0);     // Toggle OC0A on compare
    bitClear(TCCR0A, COM0A1);   // Toggle OC0A on compare
    if (CLKPR == 3)             // If clock set to 1MHz
    {
        bitSet(TCCR0B, CS00);     // Set prescaler to /1 (1uS at 1Mhz)
        bitClear(TCCR0B, CS01);
    }
    else                        // Otherwise clock set to 8MHz
    {
        bitClear(TCCR0B, CS00);
        bitSet(TCCR0B, CS01);     // Set prescaler to /8 (1uS at 8Mhz)
    }
    bitClear(TCCR0B, CS02);
    //bitSet(GTCCR,PSR10);         // Reset prescaler
    OCR0A = 49;                 // 49 + 1 = 50 microseconds (10KHz)
}
 
void loop() {}
  
