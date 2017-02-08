#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);

const byte rxAddr[6] = "00001";
int joystick[2];

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  
  radio.stopListening();
}

void loop()
{ 
  joystick[0] = analogRead(0);
  joystick[1] = analogRead(1);
  joystick[0] = map(joystick[0], 0, 1024, -500, 500);
  joystick[1] = map(joystick[1], 0, 1024, -500, 500);
  radio.write(&joystick, sizeof(joystick));
  
  delay(10);
}
