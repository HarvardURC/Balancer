// necessary includes
#include "DualVNH5019MotorShield.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <SSD1306_text.h>

// OLED setup
#define OLED_RESET 1
SSD1306_text display(OLED_RESET);

// define joystick pins
#define UP_DOWN_IN_PIN   3 
#define  LEFT_RIGHT_IN_PIN  5

// encoder pins for interrupts
#define SPD_INT_L A2
#define SPD_PUL_L A1
#define SPD_INT_R A3
#define SPD_PUL_R A4

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

// PID object and required PID globals
double output, pitch, setPoint, send_pitch;
 
float kP = 3.3;//6.3; //6.9;
float kI = 0.0;//0.02;//0.02;
float kD = 0.07;//0.10;//0.11;
float kPwheel = 1.5;
float kDwheel = 0;//0.05;

PID pid(&pitch, &output, &setPoint, kP, kI, kD, AUTOMATIC);

// globals
int Speed_L = 0;
int Speed_R = 0;
int last_countL = 0;
int last_countR = 0;

int right_stick, left_stick;
float turn_val;
int potPin = A0;
long potVal = 0.0;
float y_val = 0;
bool stop_flag = false; 

// Compensate for motor slack range 
// (low PWM values which result in no motor engagement)
float MOTORSLACK_M1 = 5;	
float MOTORSLACK_M2 = 8;


/* "center" defines the joystick value that is read when the balancer
 *  is at 0 degrees. Not precisely 0 because of uneven weight
 *  distribution. -1325 -> -1.325 degrees
 *  "delta" defines the max/min degrees the robot will tilt when moving
 *  in this case 4 degrees (4000 -> 4.000 degrees)
 */
#define center -1325
#define delta 4000

void setup()
{
	// Serial Setup
	Serial.begin(115200);
	Serial.setTimeout(10);

	// RC Receiving Pins Setup
	pinMode(3, INPUT); // Set our input pins as such
	pinMode(5, INPUT);
	pinMode(0, INPUT);

	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
	pinMode(A4, INPUT);
	pinMode(A5, OUTPUT);	

	// enable pull-up resistors
	digitalWrite(A1, HIGH);
	digitalWrite(A2, HIGH);
	digitalWrite(A3, HIGH);
	digitalWrite(A4, HIGH);
	digitalWrite(A5, HIGH);

	// create ISR's (interrupt service routines) 
	attachInterrupt(SPD_PUL_L, Encoder_L, CHANGE);
 	attachInterrupt(SPD_PUL_R, Encoder_R, CHANGE);


	// OLED setup
    display.init();
    display.clear();
    
    // dualvnh lib for motor controller
	md.init();	

	Serial.println("Orientation Sensor Raw Data Test");
	Serial.println("");

	/* Initialize the IMU (exmaple code) */
	if (!bno.begin())
	{
	/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("No BNO055 detected, check wiring / I2C ADDR");
		while (1);
	}

	bno.setExtCrystalUse(true);

	// our setpoint for the pid loop
	setPoint = .10 ;

	// Arduino PID Library setups
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-255, 255);
	pid.SetSampleTime(10);
}

void loop()
{
	// read the pulse width of each channel
	left_stick = pulseIn(5, HIGH);
	right_stick = pulseIn(3, HIGH);

	// robot should be stopped unless told otherwise
	stop_flag = true;

	// robot is allowed to balance at non-0 degree angles
	if (pulseIn(0, HIGH, 25000) < 1500)
	{
		stop_flag = false;
	}

	// map input joystick values to desired setpoint values
	left_stick = map(left_stick, 950, 2050, center - delta, center + delta);
	
	// deadzone 
	int error = 65;
	if(left_stick < (center + error) && left_stick > (center - error))
	{
		left_stick = center;
	}
	
	setPoint = left_stick;

	// map right joystick for turning (set a difference in L vs R motor values) 
	y_val = (map(right_stick, 950,2050,-15000.0, 15000));
	
	setPoint = setPoint / 1000.0;
	y_val = y_val / 1000.0;

	
	potVal = analogRead(potPin);
	
	// uncomment 1 of below to tune the respective values with potentiometer
//	kP = map(potVal, 0, 1023, 0, 10000);
//	kP = kP / 1000.0;
//	kD = map(potVal, 0, 1023, 0, 1000);
//	kD = kD / 1000.0;
//	kI = map(potVal, 0, 1023, 0, 100);
//	kI = kI / 1000.0;
//	pid.SetTunings(kP, kI, kD);
//	setPoint = map(potVal, 0, 1023, -3000, 3000);
//	setPoint = setPoint / 1000.0; 

	// IMU values
	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

	// get the z orientation
	pitch = euler.y();

	// if we're out of control stop motors
	if (abs(pitch) > 45)
	{
		md.setM1Speed(0);
		md.setM2Speed(0);
	}
	else
	{
		// use pid loop to calculate output
		pid.Compute(); 
		
		// if we're supposed to be moving, factor in the wheel constants
		double output1 = compensate_slack(-output, 1); //M1
		double output2 = compensate_slack(-output, 0); //M2
		
		if (!stop_flag)
		{
			// average of two encoder rates
			float speed_avg = (Speed_R + Speed_L) / 2;
			// final output as combination of IMU PID and encoder PID
			output1 = output1 + 
				(kPwheel * Speed_L) + (kDwheel * abs(Speed_L - last_countL));
			output2 = output2 + 
				(kPwheel * Speed_R) + (kDwheel * abs(Speed_R - last_countR));
				
			md.setM1Speed(output1);
			md.setM2Speed(output2);
		}
		
		// for derivative next loop
		last_countL = Speed_L;
		last_countR = Speed_R;

		// if we're supposed to be stationary, ignore encoder readings
		if (stop_flag) 
		{
			md.setM1Speed(output1);
			md.setM2Speed(output2);
		}

		//overwrite stationary command if we want to turn
		if (y_val > 1.0) 
		{
			md.setM1Speed(output1 + 25);
			md.setM2Speed(output2 - 25);
		}
		else if (y_val < -1.0)
		{
			md.setM1Speed(output1 - 25);
			md.setM2Speed(output2 + 25);
		}
		
	}
	// debug 
	/*
	display.setCursor(0,40);
    display.setTextSize(2,1);
    display.print(setPoint,3);

    display.setCursor(4,40);
    display.print(kP,3);
    
    display.setCursor(6,40);
    display.print(pitch,3);

    display.print(y_val,3);
	*/

	// reset encoder rates
	Speed_L = 0;
	Speed_R = 0;
}

/* CREDIT ManpreetSingh80 on Github
https://github.com/ManpreetSingh80/CHAPPIE/blob/master/SelfBalance_robot0
_66_withoutEEPROM/SelfBalance_robot0_66_withoutEEPROM.ino
*/
double compensate_slack(double Output, bool A)
{
// Compensate for DC motor non-linear "dead" zone around 0 where small values 
// don't result in movement
// Output is desired output before compensation, the boolean lets us know which 
// motor's slack to use
	if (A)
	{
		if (Output > 0)
		{
			Output += MOTORSLACK_M1;
		}
		else
		{
			Output -= MOTORSLACK_M1;
		}
	}
	else
	{
		if (Output > 0)
		{
			Output += MOTORSLACK_M2;
		}
		else
		{
			Output -= MOTORSLACK_M2;
		}
	}

	// ensure output remains within bounds of the motor sets
	Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX);
	return Output;
}

void Encoder_L()   //car up is positive car down  is negative
{
  if (digitalRead(SPD_PUL_L) == digitalRead(SPD_INT_L))
    Speed_L++;
  else
    Speed_L--;
}


void Encoder_R()    //car up is positive car down  is negative
{
  if (digitalRead(SPD_PUL_R) == digitalRead(SPD_INT_R))
    Speed_R++;
  else
    Speed_R--;
}
