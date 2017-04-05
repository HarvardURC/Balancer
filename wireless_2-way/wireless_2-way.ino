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

struct payload_t
{
   double temps[1];
};

payload_t payload;

void setup()
{
	button_state = 1;
	past_button = -1;
	Serial.begin(115200);
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
		bool done = false;
		char buffer[5];              

		while (!done)
		{
			
     		radio.read(buffer, 5);
     		done = true;
		// buffer to store payload

		// reading payload
		//radio.read(&payload, sizeof(payload) );
	//	for (int i = 0; i < 1; i++)
	//	{
	//		Serial.println(payload.temps[i]);
	//	}
		}
		Serial.println(buffer);
	}
}


void transmit()
{

	float value = analogRead(A0);
  //Serial.println(value);
  Serial.println("HIIIII");
    value = mapfloat(value, 0,1023, -3,2.6);
      Serial.println("BYYYYYYE");

  String test = (String(value));
 // Serial.println(test);
 Serial.println("Begin analogRead");
  int currentanalog = analogRead(A0);
   Serial.println("End analogRead");

  int previousanalog;
   // get incoming serial data (until newline)
        char buffer[5];         
                Serial.println("Reading string until");
     
        String setPoint = Serial.readStringUntil('\n');
                    Serial.println("End string until");

        test.toCharArray(buffer, 5);
        // can't listen while writing 

        radio.stopListening(); 
        radio.openWritingPipe(pipe);
                 Serial.println("Writing buffer");

        radio.write(buffer, 5);
                         Serial.println("END Writing buffer");

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

