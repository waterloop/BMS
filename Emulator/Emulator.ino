#include<SPI.h>

volatile boolean received;
volatile int slaveReceive, slaveSend;

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello World");
  pinMode(MISO, OUTPUT);
  /* Turn on SPI in Slave Mode */
  SPCR |= _BV(SPE);
  received = false;
  /* Interupt ON is set for SPI Commnunication */
  SPI.attachInterrupt();
}

/* SPI ISR */
ISR (SPI_STC_vect)
{
  /* Value received from master is stored in variable slaveReceive */
  slaveReceive = SPDR;
  Serial.println(slaveReceive);
  received = true;
  slaveSend = 0x003;
  /* Send the slaveSend value to master via SPDR */
  SPDR = slaveSend;
}

void loop()
{

}
