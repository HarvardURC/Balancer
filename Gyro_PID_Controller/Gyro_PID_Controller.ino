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


float kP = 10;   // crrent best @ setpoint 0.3
float kI = 0.0;
float kD = 0.48;

// -----------------------------------------

//float kP = 10.9;   // crrent best @ setpoint 0.1, with heigher base
//float kI = 0.01;
//float kD = 0.4;

PID pid(&pitch, &output, &setPoint, kP, kI, kD, AUTOMATIC);



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

float y_val;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

void setup()
{
	Serial.begin(115200);  // serial for debug

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
	setPoint = 0.35 ; //BETTER @ 3.5? 3.2 is 0, 2.9 used to be 0

	// Arduino PID Library setups
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-255, 255);
	pid.SetSampleTime(10);

	// radio setup
	radio.begin();
	radio.setRetries(1, 2); // delay of 1ms, 2 retries
	radio.openReadingPipe(1, pipe);
	radio.startListening();
	radio.printDetails();

}

void loop()
{
	currentMillis = millis();	// get system time

	// make sure we only compute pid and set motors at correct interval (our cycle is 10 ms)
	// see: https://learn.adafruit.com/multi-tasking-the-arduino-part-1/using-millis-for-timing

	if (currentMillis - previousMillis >= interval)
	{
		previousMillis = currentMillis;

		// get new gyro data
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
				md.setM1Speed(output + y_val ); //26.5
				md.setM2Speed(-output ); // 25
			}
			else 
			{
				md.setM1Speed(output + y_val ); //26.5
				md.setM2Speed(-output ); // 25
			}
		
		}
	}

	listen_();
	transmit();
	//Serial.println(setPoint);

}

void listen_() 
{
	if (radio.available() )
	{	
     	radio.read(&payload, sizeof(payload_t));
		// buffer to store payload
		setPoint = (payload.x_val);
		y_val = (payload.y_val);

	}
}

void transmit()
{

// for more detailed debug
//	imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//	imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//	imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

	//float temps[] = {accel.x(), accel.y(), accel.z(), gyro.x(), gyro.y(), gyro.z(), mag.x(), mag.y(), mag.z(), pitch, setPoint, output, kP, kI, kD};
	// can't listen while writing
	radio.stopListening();
	radio.openWritingPipe(pipe);

	payload_t payload;

	//char buff[transmit_buffer];              
	//dtostrf(pitch, 5, 3, buff);
	//radio.write(buff,transmit_buffer);
	//memcpy(payload.x_val, buff, transmit_buffer );


	radio.openReadingPipe(1,pipe);
	// begin listening again
	radio.startListening();
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

