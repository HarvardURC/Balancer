#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);

const byte rxAddr[6] = "00001";

void setup()
{
  while (!Serial);
  Serial.begin(9600);
  
  radio.begin();
  radio.openReadingPipe(0, rxAddr);
  
  radio.startListening();
}

void loop()
{
  if (radio.available())
  {
    int joystick[2] = {0};
    radio.read(&joystick, sizeof(joystick));
    
    Serial.print(joystick[0]);
    Serial.print(" ");
    Serial.println(joystick[1]);
  }
}

