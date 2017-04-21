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


// 8-11 posible
float kP = 8.8;
float kI = 0.0;
float kD = 0.2;
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
float y_val = 0;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

#define center -1050
#define delta 2000

void setup()
{
	// Serial Setup
	Serial.begin(115200);  // serial for debug
	Serial.setTimeout(10);

	// RC Receiving Pins Setup
	pinMode(3, INPUT); // Set our input pins as such
	pinMode(5, INPUT);

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
	setPoint = -1.2 ; //BETTER @ 3.5? 3.2 is 0, 2.9 used to be 0

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
	//m1 positive motor backlash = 24
	// m2 positive motor backlash = 26

	
	md.setM1Speed(-24); //M1 is more powerful motor
	md.setM2Speed(-200); //M1 is more powerful motor
	//md.setM1Speed(8); //M1 is more powerful motor
	//md.setM2Speed(9); //M1 is more powerful motor

}
  
