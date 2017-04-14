#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define transmit_buffer 10
#define receive_buffer 10


// http://maniacbug.github.io/RF24/classRF24.html#a391eb0016877ec7486936795aed3b5ee
// radio variables
RF24 radio(9, 10);
const uint64_t pipe = 0xF0F0F0F0AA;

// joystick variables
int VRx = A0;
int VRy = A1;
int x_pos, y_pos, past_x_pos, past_y_pos, button_state, past_button;
int button_pin = 7;

struct payload_t
{
	float x_val;
	float y_val;
};


void setup()
{
	Serial.begin(115200);
	Serial.setTimeout(10);
	// radio setup
 	radio.begin();
  	radio.setRetries(2, 5); // delay of 15ms, 10 retries
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
		payload_t payload;
		
     	radio.read(&payload, sizeof(payload_t));
		Serial.println(payload.x_val);
	}
}


void transmit()
{
	payload_t payload;

	float straight = analogRead(A0);
	float pivot_turn = analogRead(A1);

		//Serial.println(value);
	straight = mapfloat(straight, 1023, 0,-11, 7);
	pivot_turn = mapfloat(pivot_turn, 1023, 0, -25, 25);
	payload.x_val = straight;
	payload.y_val = pivot_turn;
        // can't listen while writing 
        radio.stopListening(); 
        radio.openWritingPipe(pipe);
        radio.write(&payload, sizeof(payload_t));
        // done writing, back to reading pipe
        radio.openReadingPipe(1,pipe); 
        // begin listening again
        radio.startListening();
        // not spamming comms
}

long mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

