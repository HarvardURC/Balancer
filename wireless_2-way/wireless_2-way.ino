#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// http://maniacbug.github.io/RF24/classRF24.html#a391eb0016877ec7486936795aed3b5ee
// radio variables
RF24 radio(9, 10);
const uint64_t pipe = 0xF0F0F0F0AA;

// joystick variables
int VRx = A0;
int VRy = A1;
int x_pos, y_pos, past_x_pos, past_y_pos, button_state, past_button;
int button_pin = 7;

void setup()
{
	button_state = 1;
	past_button = -1;
	Serial.begin(115200);
	// radio setup
 	radio.begin();
  	radio.setRetries(15, 10); // delay of 15ms, 10 retries
  	radio.openReadingPipe(1, pipe);
  	radio.startListening(); 
  	radio.printDetails();  

  	// joystick setup
	pinMode(VRx, INPUT);
    pinMode(VRy, INPUT);
    
  	// pull-up resistor joystick button
  	pinMode(button_pin, INPUT_PULLUP); 
 }


void loop()
{
	listen_();
	delay(10);
	transmit();
}

// listen for incoming data
void listen_() 
{
	if (radio.available() )
	{	
		// buffer to store payload
		char command[32] = {0};

		// reading payload
		radio.read(&command, sizeof(command) );
		Serial.println(command);
	}
}

void transmit()
{
  //x_pos = analogRead(VRx);
  //Serial.println(x_pos);
  button_state = digitalRead(button_pin);
  
  // y_pos = analogRead(VRy);
  // something new to write
  //if (x_pos != past_x_pos || y_pos != past_y_pos)
 // {
 
  //	past_x_pos = x_pos;
 // 	past_y_pos = y_pos;

	if (button_state != past_button)
	{
		past_button = button_state;
  		const char text[] = "Hi balancer";
	
	  	// can't listen while writing 
	  	radio.stopListening(); 
	  	radio.openWritingPipe(pipe);
	  	radio.write(&text, sizeof(text) );
	  	// done writing, back to reading pipe
	    radio.openReadingPipe(1,pipe); 
	    // begin listening again
	    radio.startListening();
	    // not spamming comms
	    delay(20);
	}

}


