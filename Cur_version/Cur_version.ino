//TODO: Turning is bizarre... almost "incrememnts" every time hit throttle in that direction
// TODO: ALso, to get rid of crazy mvmt get rid of print statemnets
#include "DualVNH5019MotorShield.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <SPI.h>
#include <avr/dtostrf.h>
#define transmit_buffer 10
#define receive_buffer 10
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SSD1306_text.h>

#define OLED_RESET 1
SSD1306_text display(OLED_RESET);

#define UP_DOWN_IN_PIN   3   //maybe flip these 2
#define  LEFT_RIGHT_IN_PIN  5

#define SPD_INT_L A2   //interrupt 
#define SPD_PUL_L A1   
#define SPD_INT_R A3   //interrupt 
#define SPD_PUL_R A4   

int Speed_L = 0;
int Speed_R = 0;
int last_count = 0;
// http://maniacbug.github.io/RF24/classRF24.html#a391eb0016877ec7486936795aed3b5ee
// radio variables
const uint64_t pipe = 0xF0F0F0F0AA;

int right_stick, left_stick;
float turn_val;
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

// NEWEST BESTEST W/ R?C
// 8-11 possible kp w/o slack
// ?-? possible kpw/ slack
//float kP = 6.3;
//float kI = 0.0;
//float kD = 0.18;
// NEWEST BESTEST W/ R?C
// 8-11 possible kp w/o slack
// ?-? possible kpw/ slack
// OLD BEST :
 // kP = 6.35
 // kI = 0
 // kD = 0.07
// CURRENT: 
/*
kp = 6.3
kI = 0.0
kD 0.07

*/
 
float kP = 3.3;//6.3; //6.9;
float kI = 0.0;//0.02;//0.02;
float kD = 0.07;//0.10;//0.11;
float kPwheel = 3.0;
float kDwheel = 0;
// ----------------
// -----------------------------------------

PID pid(&pitch, &output, &setPoint, kP, kI, kD, AUTOMATIC);


int potPin = A0;
long potVal = 0.0;
// was 65/60
//float MOTORSLACK_1 = 26.5;	// Compensate for motor slack range (low PWM values which result in no motor engagement)
//float MOTORSLACK_2 = 25;	// Compensate for motor slack range (low PWM values which result in no motor engagement)

float MOTORSLACK_M1 = 5;	// Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_M2 = 8;	// Compensate for motor slack range (low PWM values which result in no motor engagement)


// timer setup: we're running on a 10 ms loop
long previousMillis = 0;
unsigned long currentMillis;
long interval = 10;
float y_val = 0;
bool stop_flag = false; 

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

#define center -1325
#define delta 4000

void setup()
{
	// Serial Setup
	Serial.begin(115200);  // serial for debug
	Serial.setTimeout(10);

	// RC Receiving Pins Setup
	pinMode(3, INPUT); // Set our input pins as such
	pinMode(5, INPUT);
	pinMode(0, INPUT);

	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
	pinMode(A4, INPUT);
	digitalWrite(A1, HIGH);
	digitalWrite(A2, HIGH);
	digitalWrite(A3, HIGH);
	digitalWrite(A4, HIGH);

	pinMode(A5, OUTPUT);	
	digitalWrite(A5, HIGH);

	attachInterrupt(SPD_PUL_L, Encoder_L, CHANGE);
 	attachInterrupt(SPD_PUL_R, Encoder_R, CHANGE);


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
	setPoint = .10 ; //BETTER @ 3.5? 3.2 is 0, 2.9 used to be 0

	// Arduino PID Library setups
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-255, 255);
	pid.SetSampleTime(10);

	// radio setup
	Serial.println("BEGIN"); 
}
// get new gyro data


void loop()
{
	//950 -> 2050
	left_stick = pulseIn(5, HIGH); // each channel
	right_stick = pulseIn(3, HIGH); // Read the pulse width of 
	stop_flag = true;
	if (pulseIn(0, HIGH, 25000) < 1500)
	{
		stop_flag = false;
	}

	left_stick = map(left_stick, 950, 2050, center - delta, center + delta);
	int error = 50;
	if(left_stick < (center + error) && left_stick > (center - error))
	{
		left_stick = center;
	}
	
	setPoint = left_stick;
	y_val = (map(right_stick, 950,2050,-15000.0, 15000));
	setPoint = setPoint / 1000.0;
	y_val = y_val / 1000.0;

	
	potVal = analogRead(potPin);
//	kP = map(potVal, 0, 1023, 0, 10000);
//	kP = kP / 1000.0;
//	kD = map(potVal, 0, 1023, 0, 1000);
//	kD = kD / 1000.0;
//	kI = map(potVal, 0, 1023, 0, 100);
//	kI = kI / 1000.0;
//	pid.SetTunings(kP, kI, kD);

//	setPoint = map(potVal, 0, 1023, -3000, 3000);
//	setPoint = setPoint / 1000.0; 
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
			// if we're supposed to be moving, factor in the wheel constants
			double output1 = compensate_slack(-output, 1); //M1
			double output2 = compensate_slack(-output, 0); //M2
			if (!stop_flag)
			{
				//Serial.println("HI");
				output = output + (kPwheel * Speed_R) + (kDwheel * abs(Speed_R - last_count));
				 md.setM1Speed(output1);
				 md.setM2Speed(output2);
				 display.setCursor(4,40);
 		         display.print(Speed_R );
 		         Serial.println(Speed_R);
						
			}
			last_count = Speed_R;
		//	Serial.println(y_val);
			if (stop_flag) // we're supposed to be stationary
			{
				md.setM1Speed(output1);
				md.setM2Speed(output2);
			}
			if (y_val > 1.0) //overwrite stationary with a turn
			{
				Serial.println(">1");
				//double output1 = compensate_slack((-output - y_val), 1); //M1
				md.setM1Speed(output1-25); //26.5
				md.setM2Speed(output2 + 25); // 25
			}
			else if (y_val < -1.0)
			{
								Serial.println("<-1");
								Serial.println("<-1");

				//double output2 = compensate_slack( (output - y_val), 0); //M2
				md.setM1Speed(output1 + 25); //26.5
				md.setM2Speed(output2 - 25); // 25
			}
			Serial.println(y_val);
			
		//	else
		//	{
				/*int speed_dif = abs(Speed_L) - abs(Speed_R);
				//double output1 = compensate_slack(-output, 1); //M1
				//double output2 = compensate_slack(output, 0); //M2
				if (Speed_L > Speed_R)
				{
					md.setM1Speed(-output + speed_dif);
					md.setM2Speed(-output - speed_dif);
				}
				else if (Speed_R > Speed_L)
				{
					md.setM1Speed(-output - speed_dif);
					md.setM2Speed(-output + speed_dif);
				}
				else
				{*/
	//				md.setM1Speed(output1);
	//				md.setM2Speed(output2);
			//	}
				// 25
	/*			            display.setCursor(3,40);
                  display.print(output1,3);
                  display.setCursor(6,40);
                  display.print(output2,3);
*/
			//}
		}
	display.setCursor(0,40);
     display.setTextSize(2,1);
     display.print(setPoint,3);
/*
      display.setCursor(4,40);
      display.print(kP,3);
      */
 display.setCursor(6,40);
      display.print(pitch,3);
     //   Serial.println(Speed_L);

      //display.print(y_val,3);
      //
	Speed_L = 0;
	Speed_R = 0;
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
long mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void Encoder_L()   //car up is positive car down  is negative
{
  if (digitalRead(SPD_PUL_L) == digitalRead(SPD_INT_L))
    Speed_L++;
  else
    Speed_L--;
 // Serial.println(Speed_L);
 //  Serial.print("SPEED_L:    ");
   // Serial.println(Speed_L);
}


void Encoder_R()    //car up is positive car down  is negative
{
  if (digitalRead(SPD_PUL_R) == digitalRead(SPD_INT_R))
    Speed_R++;
  else
    Speed_R--;
    
  // Serial.print("SPEED_R:    ");
   // Serial.println(Speed_R);
}


