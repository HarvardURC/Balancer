// TODO: check our whole reversal isn't messing with the pid
	// (see note in code where we declare enc_direction)
	// we could just leave it up to the pid to set + / -, which seems like
	// a better idea. Won't change this until back and able to test
	// (so we don't push defective code)
// TODO: replace DUALVNH5019 lib
// TODO: seperate constants for each motor
// TODO: quaternions instead of euler vectors?
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

double output, input, setPoint;

PID pid(&input, &output, &setPoint, 5.3, 0, 0.18, AUTOMATIC);

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
		input = event.orientation.z;

		/* SEE TODO- might just get rid of this and let pid calculate +/- */
		// if z has changed update pos. and find which direction we rotated
		int enc_direction;
		if (input < 0)
		{
			enc_direction = 1;
		}
		if (input > 0)
		{
			enc_direction = -1;
		}
		input = abs(input);

		// if we're out of control stop motors.
		if (input > 45)
		{
			md.setM1Speed(0);
			md.setM2Speed(0);
		}
		else
		{
			// commented out option to create a deadzone when robot within 1.5 deg of ctr
			/*
			if (input < 1.5)
			{
				md.setM1Speed(0);
				md.setM2Speed(0);
			    }
			else
			{
				Serial.println(output);
			*/


			pid.Compute(); // use pid loop to calculate output

			// experimental code, fixing motor "inequality" in either direction
			// (turns more easily in one direction than the other)
			/* if (event.orientation.z < 0)
			{
				double output1 = compensate_slack(output, 1) + 15;
				double output2 = compensate_slack(output, 0) + 15;
			}*/
			double output1 = compensate_slack(output, 1); //M1
			double output2 = compensate_slack(output, 0); //M2

			md.setM1Speed(-enc_direction * output1);
			md.setM2Speed(enc_direction * output2);


			// if using deadzone, need this close bracket
			//}
		}
	}

	// testing + debugging data
	/*Serial.print(" Z: ");
	  Serial.print(input);
	  Serial.print(" output: ");
	  Serial.print(output);
	  Serial.print(" setPoint: ");

	  Serial.print(setPoint);
	  Serial.print(" end dir: ");
	  Serial.println(enc_direction);*/

	//  Serial.print(millis()); // print the time in milliseconds since the program started
	//  Serial.print(',');
	//  Serial.println(euler.z()); // print to serial

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

