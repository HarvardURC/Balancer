#include "DualVNH5019MotorShield.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <avr/dtostrf.h>
#define transmit_buffer 10
#define receive_buffer 10
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SSD1306_text.h>

#define OLED_RESET 1
SSD1306_text display(OLED_RESET);

// http://maniacbug.github.io/RF24/classRF24.html#a391eb0016877ec7486936795aed3b5ee
// radio variables
RF24 radio(3, 5);
const uint64_t pipe = 0xF0F0F0F0AA;


/*
  CREDIT: ARDUINO BNO055 EXAMPLE CODE
*/

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (10)

// PID match PWM max in reverse and foward
#define BALANCE_PID_MIN -255
#define BALANCE_PID_MAX 255

// Need to call constructor differently for Arduino zero
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// normal constructor call:
// Adafruit_BNO055 bno = Adafruit_BNO055();

DualVNH5019MotorShield md;

/* PID object and required globals */

double output, pitch, setPoint, send_pitch;

// pid consts used with serial input to modify constants

//float kP = 7.3;   // crrent best @ setpoint 0.6
//float kI = 0.01;
//float kD = 0.5;


//float kP = 8.7;   // crrent best @ setpoint 0.6
//float kI = 0.01;
//float kD = 0.51;

//float kP = 9.5;   // crrent best @ setpoint 0.6
//float kI = 0.01;
//float kD = 0.55;

//float kP = 10;   // crrent best @ setpoint 0.2
//float kI = 0.01; //OSCILLATING AROUND CTR
//float kD = 0.5;


//float kP = 10;   // crrent best @ setpoint 0.3
//float kI = 0.0;
//float kD = 0.48;

//---------------------new thumper BEFORE LCD
//float kP = 8.1;
//float kI = 0.0;
//float kD = 0.15;

//float kP = 9.3;
//float kI = 0.0;
//float kD = 0.4;

//float kP = 7.0;
//float kI = 0.0;
//float kD = 0.3;

//float kP = 9.0;
//float kI = 0.0;
//float kD = 0.35;

//float kP = 9.6;
//float kI = 0.0;
//float kD = 0.38;

float kP = 9.6;
float kI = 0.0;
float kD = 0.38;
// ----------------
// -----------------------------------------

PID pid(&pitch, &output, &setPoint, kP, kI, kD, AUTOMATIC);


int potPin = 0;
long potVal = 0.0;
// was 65/60
//float MOTORSLACK_1 = 26.5;	// Compensate for motor slack range (low PWM values which result in no motor engagement)
//float MOTORSLACK_2 = 25;	// Compensate for motor slack range (low PWM values which result in no motor engagement)

float MOTORSLACK_1 = 30;	// Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_2 = 30;	// Compensate for motor slack range (low PWM values which result in no motor engagement)


// timer setup: we're running on a 10 ms loop
long previousMillis = 0;
unsigned long currentMillis;
long interval = 10;

struct payload_t
{
	float x_val;
	float y_val;
};

payload_t payload;

float y_val = 0;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/


void setup()
{
	Serial.begin(115200);  // serial for debug
	Serial.setTimeout(10);

	// OLED setup
    display.init();
    display.clear();                 // clear screen

	md.init();	// dualvnh lib for motor controller

	Serial.println("Orientation Sensor Raw Data Test");
	Serial.println("");

	/* Initialize the sensor */
	if (!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while (1);
	}

	bno.setExtCrystalUse(true);

	// our setpoint for the pid loop
	setPoint = -2.3 ; //BETTER @ 3.5? 3.2 is 0, 2.9 used to be 0

	// Arduino PID Library setups
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-255, 255);
	pid.SetSampleTime(10);

	// radio setup
	radio.begin();
	radio.setRetries(2, 3); // delay of 2ms, 3 retries
	radio.openReadingPipe(1, pipe);
	radio.startListening();
	Serial.println("BEGIN"); 
}
// get new gyro data


void loop()
{
	potVal = analogRead(potPin);
	kP = map(potVal, 0, 1023, 0, 20000);
	kP = kP / 1000.0;
	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

		// get the z orientation
		pitch = euler.y();

		// if z has changed update pos. and find which direction we rotated

		// if we're out of control stop motors.
		if (abs(pitch) > 45)
		{
			md.setM1Speed(0);
			md.setM2Speed(0);
		}
		else
		{
			pid.Compute(); // use pid loop to calculate output

			//double output1 = compensate_slack(output, 1); //M1
			//double output2 = compensate_slack(output, 0); //M2
		//	Serial.println(y_val);
			if (y_val > 0)
			{
				md.setM1Speed(-output + y_val ); //26.5
				md.setM2Speed(output ); // 25
			}
			else 
			{
				md.setM1Speed(-output + y_val ); //26.5
				md.setM2Speed(output ); // 25
			}
		
		}

	listen_();
	//transmit();
	//	radio.printDetails();

	//Serial.println(pitch);
	//Serial.println(setPoint);
      display.setCursor(6,40);
      display.setTextSize(2,1);
     display.print(kP,3);
}

void listen_() 
{
	if (radio.available() )
	{	
		Serial.println("IN LISTEN");

     	radio.read(&payload, sizeof(payload_t));
		// buffer to store payload
		setPoint = (payload.x_val);
		y_val = (payload.y_val);
	}
}

void transmit()
{
	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

	float new_pitch = euler.y();
	if (new_pitch != pitch)
	{
		// can't listen while writing
		radio.stopListening();
		radio.openWritingPipe(pipe);
	
		payload_t payload2;
		payload2.x_val = pitch;
		payload2.y_val = 0.0;
	
		//char buff[transmit_buffer];              
		//dtostrf(pitch, 5, 3, buff);
		//radio.write(buff,transmit_buffer);
		//memcpy(payload.x_val, buff, transmit_buffer );
	
		radio.write(&payload2, sizeof(payload_t));
	
		radio.openReadingPipe(1,pipe);
	
		// begin listening again
		radio.startListening();
	}
}



/*CREDIT ManpreetSingh80 on Github
   https://github.com/ManpreetSingh80/CHAPPIE/blob/master/SelfBalance_robot0_66_withoutEEPROM/SelfBalance_robot0_66_withoutEEPROM.ino
*/
double compensate_slack(double Output, bool A)
{
	// Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
	// Output is desired output before compensation, the boolean lets us know which motor's slack to use

	if (A)
	{
		if (Output > 0)
		{
			Output += MOTORSLACK_1;
		}
		else
		{
			Output -= MOTORSLACK_1;
		}
	}
	else
	{
		if (Output > 0)
		{
			Output += MOTORSLACK_2;
		}
		else
		{
			Output -= MOTORSLACK_2;
		}
	}

	// ensure output remains within bounds of the motor sets
	Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX);
	return Output;
}
long mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

