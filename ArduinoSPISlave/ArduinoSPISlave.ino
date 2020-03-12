// Written by Nick Gammon
// February 2011
/**
 * Send arbitrary number of bits at whatever clock rate (tested at 500 KHZ and 500 HZ).
 * This script will capture the SPI bytes, when a '\n' is recieved it will then output
 * the captured byte stream via the serial.
 */

#include <SPI.h>
#define LENGTH 14

byte buf[LENGTH];
volatile boolean reading;
volatile boolean finish_receive = false;
int pos = 0;


void setup (void)
{
  pinMode(10, OUTPUT);
  Serial.begin (115200);   // debugging

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
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
  Serial.println(c);
  if (c == 0) {
    reading = false;
    finish_receive = true;
  }
  if (reading) {
      if (pos < LENGTH) buf[pos++] = c;
  }
  if (c == 255) reading = true;
}  // end of interrupt routine SPI_STC_vect

// main loop - wait for flag set in interrupt routine
void loop (void)
{
  if (finish_receive) {
   /* for (int i = 0; i < LENGTH; ++i) {
      Serial.print(buf[pos]);
      Serial.print(" ");
    }
    Serial.println("");*/
    finish_receive = false;
  }
  
  delay(100);
}
