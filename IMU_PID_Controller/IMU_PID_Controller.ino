// note: to use serial in to change pid constants mid-test,
	// format to enter the values in:
	//kP,kI,kD

// TODO: replace DUALVNH5019 lib
// TODO: seperate constants for each motor
// TODO: wire connection guide somewhere?
// TODO: Gyro Offsets

// NOTE MIN motor speed to freely rotate ~68
// NOTE avoid using pins with LEDs attached

#include "DualVNH5019MotorShield.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

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

/*float kP = 5.3;
float kI = 0;
float kD = 0.18;
*/

//float kP = 28;
//float kI = 0;
//float kD = 1.5;

//float kP = 28;
//float kI = 0;
//float kD = 1.3;

//float kP = 28.5;
//float kI = 0;
//float kD = 1.2;


//float kP = 29.6;
//float kI = 0;
//float kD = 1.2;


//float kP = 30.2;
//float kI = 0;
//float kD = 0;


//float kP = 34.5;
//float kI = 0;
//float kD = 0.9;


//float kP = 38;
//float kI = 0;
//float kD = 0.9;

float kP = 38;
float kI = 0;
float kD = 0.9;

PID pid(&pitch, &output, &setPoint, kP, kI, kD, AUTOMATIC);



// was 65/60
float MOTORSLACK_1 = 65;	// Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_2 = 55;	// Compensate for motor slack range (low PWM values which result in no motor engagement)

// timer setup: we're running on a 10 ms loop
long previousMillis = 0;
unsigned long currentMillis;
long interval = 10;

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

	// our setpoint for the pid loop
	setPoint = 0;

	// Arduino PID Library setups
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-255, 255);
	pid.SetSampleTime(10);

}

void loop()
{
	currentMillis = millis();	// get system time

	// make sure we only compute pid and set motors at correct interval (our cycle is 10 ms)
	// see: https://learn.adafruit.com/multi-tasking-the-arduino-part-1/using-millis-for-timing

	if (currentMillis - previousMillis >= interval)
	{
		previousMillis = currentMillis;

		// get new sensor event
		sensors_event_t event;
		bno.getEvent(&event);

		// get the z orientation
		pitch = event.orientation.z;
		send_pitch = pitch;

		// if z has changed update pos. and find which direction we rotated
		/* REVISIT THIS let pid calculate +/- */
		int enc_direction;
		if (pitch < 0)
		{
			enc_direction = 1;
		}
		if (pitch > 0)
		{
			enc_direction = -1;
		}
		
		pitch = abs(pitch);


		// if we're out of control stop motors.
		if (pitch > 45)
		{
			md.setM1Speed(0);
			md.setM2Speed(0);
		}
		else
		{
			// commented out option to create a deadzone when robot within 1 deg of ctr
			/*
			if (pitch < 1)
			{
				md.setM1Speed(0);
				md.setM2Speed(0);
			}
			*/


			pid.Compute(); // use pid loop to calculate output


			
			// experimental code, fixing motor "inequality" in either direction
			// (turns more easily in one direction than the other)
			/* if (event.orientation.z < 0)
			{
				double output1 = compensate_slack(output, 1) + 15;
				double output2 = compensate_slack(output, 0) + 15;
			}*/
			//double output1 = compensate_slack(output, 1); //M1
			//double output2 = compensate_slack(output, 0); //M2

			md.setM1Speed(-output * enc_direction);
			md.setM2Speed(output * enc_direction);
		}
	}
	// if serial data input is avaiable (modify pid constants)
	if (Serial.available() > 0)
	{
		// get incoming serial data (until newline)
		String inSerialData = Serial.readStringUntil('\n');

		// get first comma position
		int delimIndx = inSerialData.indexOf(",");

		// get 2nd comma position
		int delimIndx2 = inSerialData.indexOf(",", delimIndx+1);

		// use substrings to get our constants
		kP = inSerialData.substring(0, delimIndx).toFloat();
		kI = inSerialData.substring(delimIndx+1, delimIndx2).toFloat();
		kD = inSerialData.substring(delimIndx2).toFloat();
		// set our new PID constantss
		pid.SetTunings(kP, kI, kD);
	}

	// graphing data
	// CREDIT: farrellf, https://github.com/farrellf/Telemetry-Viewer

	// for basic graphing of pitch vs setpoint and tuning
	// Serial.print(pitch); Serial.print(",");
	// Serial.println(setPoint);

	// for more detailed debug
	imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

	/* Explaining the below
	   Arduino does not support sprintf with floats
	   (which would have been a much better alternative to the below)
	   dtostrf is nice, but we'd have to do that for every one of the
	   variables, making lots of temporary buffers and it would have taken
	   a lot of space.
	   A user created function, as at:
	   https://gist.github.com/asheeshr/9004783
	   could work well, but I can't test until back on campus, so for now
	   the messy code below until we get the graphing visualization to work.
	*/
	Serial.print(accel.x()); Serial.print(",");
	Serial.print(accel.y()); Serial.print(",");
	Serial.print(accel.z()); Serial.print(",");
	Serial.print(gyro.x());  Serial.print(",");
	Serial.print(gyro.y());  Serial.print(",");
	Serial.print(gyro.z());  Serial.print(",");
	Serial.print(mag.x());   Serial.print(",");
	Serial.print(mag.y());	 Serial.print(",");
	Serial.print(mag.z());	 Serial.print(",");
	Serial.print(send_pitch); 	 Serial.print(",");
	Serial.print(setPoint);  Serial.print(",");
	Serial.print(output);    Serial.print(",");
	Serial.print(kP);        Serial.print(",");
	Serial.print(kI);		 Serial.print(",");
	Serial.println(kD);
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
		Output = Output + MOTORSLACK_1;
	}
	else
	{
		Output = Output + MOTORSLACK_2;
	}

	// ensure output remains within bounds of the motor sets
	Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX);
	return Output;
}

