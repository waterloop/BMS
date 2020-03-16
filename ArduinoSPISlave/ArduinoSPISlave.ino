// Written by Nick Gammon
// February 2011
/**
 * Send arbitrary number of bits at whatever clock rate (tested at 500 KHZ and 500 HZ).
 * This script will capture the SPI bytes, when a '\n' is recieved it will then output
 * the captured byte stream via the serial.
 */

#include <SPI.h>
#define LENGTH 1024

volatile byte buf[LENGTH];
volatile boolean finish_receive = false;
volatile int pos = 0;
volatile int pos_printed = 0;


void setup (void)
{
  pinMode(MOSI,INPUT);
  pinMode(SCK,INPUT);
  pinMode(MISO,OUTPUT);
  pinMode(SS, INPUT);
  Serial.begin (115200);   // debugging
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // get ready for an interrupt 
  // now turn on interrupts
  SPI.attachInterrupt();
}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;
  buf[pos++] = c;
  pos %= LENGTH;
  /*if (c == 0) {
    reading = false;
    finish_receive = true;
  }
  if (reading) {
      if (pos < LENGTH) buf[pos++] = c;
  }
  if (c == 255) reading = true;*/
}  // end of interrupt routine SPI_STC_vect

// main loop - wait for flag set in interrupt routine
void loop (void)
{
  while (pos != pos_printed) {
     Serial.println(buf[pos_printed ++]);
     pos_printed %= LENGTH;
  }
}
