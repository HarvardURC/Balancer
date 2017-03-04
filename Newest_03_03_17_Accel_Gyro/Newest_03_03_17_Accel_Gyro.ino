
// NEW CREDIT: http://robottini.altervista.org/kalman-filter-vs-complementary-filter
// http://forum.arduino.cc/index.php?topic=58048.0
#include "DualVNH5019MotorShield.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define pi 3.1415926
#include <PID_v1.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (10) // Set the delay between fresh samples

#define BALANCE_PID_MIN -255	// PID match PWM max in reverse and foward
#define BALANCE_PID_MAX 255

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29); // Need to call constructor differently for Arduino zero

DualVNH5019MotorShield md;

/* PID constants and required globals */
double output, pitch, setPoint, send_pitch;

//
int gyro_angle;
int acc_angle;
double gyro_rate;
/*
int val_sensor;
float accZ = 0;
float accY = 0;
float x1=0;
float y1=0;
float x2=0;
float y2=0;
float actAngle=0;  
float actAngleC=0;  
float actAngleRG=0;
float actAngleRA=0;
float actAngleC2=0;
*/
unsigned long current_time = 0;               //timer
unsigned long  delta_t = 0;            //delta time or how long it takes to execute data acquisition 
int lastLoopTime=5;
float x_angle=0;
float x_angleC=0;
float x_angleRG=0;
float  x_angleRA=0;
float x_angle2C=0;

float MOTORSLACK_1 = 30;	// Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_2 = 30;	// Compensate for motor slack range (low PWM values which result in no motor engagement)

float Kp = 0;   // crrent best @ setpoint 0.35
float Ki = 0;
float Kd = 0;

float tau = 0.075;
float a = 0.0;

PID pid(&pitch, &output, &setPoint, Kp, Ki, Kd, AUTOMATIC);



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
	setPoint = 0; //BETTER @ 3.5? 3.2 is 0, 2.9 used to be 0

	// Arduino PID Library setups
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-255, 255);
	pid.SetSampleTime(10);


}

void loop()
{
	delta_t = millis() - current_time;
	current_time = millis();	// get system time
	
	acc_angle = getAccAngle() * 180/pi;
	imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

	gyro_rate = gyro.x();
	
	
	

		/*
		// get new gyro data
		Filter();

		// if z has changed update pos. and find which direction we rotated
		
		// if we're out of control stop motors.
		if (abs(pitch) > 45)
		{
			md.setM1Speed(0);
			md.setM2Speed(0);
		}
		else
		{
			pid.Compute();
			double output1 = compensate_slack(output, 1); //M1
			double output2 = compensate_slack(output, 0); //M2

			md.setM1Speed(-output1); //26.5
			md.setM2Speed(output2); // 25
		}
		*/
	
}


//credit: https://codebender.cc/user/bruno.huang
/*void Filter()
{
	imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

	//credit: https://codebender.cc/user/bruno.huang
	Angle_Raw = (atan2(accel.y(), accel.z()) * 180 / pi);
	omega = gyro.x(); // + Gyr_Gain + Gyr_offset
	unsigned long now = micros();
	time_change = now - pre_time;
	pre_time = now;
	dt = time_change * 0.00001;
	Angle_Delta = (Angle_Raw - Angle_Filtered) * 0.64;
  	Angle_Recursive = Angle_Delta * dt + Angle_Recursive;
  	Angle_Confidence = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 1.6 + omega; //error bar
  	Angle_Filtered = Angle_Confidence * dt + Angle_Filtered;
  	pitch = Angle_Filtered;
	Serial.println(Angle_Filtered); 	
}
*/
/*CREDIT ManpreetSingh80 on Github
   https://github.com/ManpreetSingh80/CHAPPIE/blob/master/SelfBalance_robot0_66_withoutEEPROM/SelfBalance_robot0_66_withoutEEPROM.ino
*/
double getAccAngle()
{
	imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	return (atan2(accel.y(), accel.z()));
}

double complementary(float new_angle, float new_rate, int looptime)
{
	float dtC = float(looptime)/1000.0;
	a = tau/(tau + dtC);
	x_angleC = a* (x_angleC + new_rate * dtC) + (1-a) * (new_angle);

	return x_angleC;
}

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

