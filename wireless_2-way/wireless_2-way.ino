#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// http://maniacbug.github.io/RF24/classRF24.html#a391eb0016877ec7486936795aed3b5ee
// radio variables
RF24 radio(7, 8);
const uint64_t rxAddr = 0xF0F0F0F0AA;

// joystick variables


void setup()
{
	Serial.begin(115200);
 	radio.begin();
  	radio.setRetries(15, 10); // delay of 15ms, 10 retries
  	radio.openReadingPipe(1, rxAddr);
  	radio.startListening(); 
  	radio.printDetails();       
 }

void loop()
{
	listen();
	delay(10);
	broadcast();
}

// listen for incoming data
void listen() 
{
	if (radio.available() )
	{
		// payload status
		bool payload_receive = false;
		
		// buffer to store payload
		char text[32] = {0};

		// wait until successful payload delivery
		while (!end_receive)
		{
			// reading payload
			payload_receive = radio.read(&command, sizeof(command) );
		}
		Serial.print(command);
}


void broadcast()
{
	
}
}

