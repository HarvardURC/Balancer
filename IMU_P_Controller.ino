// TODO: begin implementing typical PID concepts (or Brett's PID library?)
// TODO: expand to PI controller
// TODO: replace DUALVNH5019 lib
// TODO: seperate constants for each motor
// TODO: quaternions instead of euler vectors?
// TODO: wire connection guide somewhere?


/* 
 *CREDIT: ARDUINO BNO055 EXAMPLE CODE
 */
#include "DualVNH5019MotorShield.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)


// NEED TO DO THIS WITH THE ARDUINO ZERO (calling constructor differently)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

/* normal constructor call*/
/* Adafruit_BNO055 bno = Adafruit_BNO055(); */

DualVNH5019MotorShield md;

#include <Encoder.h>

/* avoid using pins with LEDs attached */
Encoder myEnc(3, 5);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

void setup() 
{
  
  Serial.begin(9600);
  md.init();
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialize the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

}

long oldPosition  = -999;

void loop() 
{

   // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  /* euler vector setup from IMU*/
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(" Z: ");

  /* euler vector of z coordinate (our "pitch") */
  float newPosition = euler.z();
  Serial.println(newPosition);

  /* if z has changed update pos. and find which direction we rotated */
  if (newPosition != oldPosition) 
  {
    oldPosition = newPosition;

    int enc_direction;
    if (newPosition < 0)
    {
       enc_direction = 1;
    }
    if (newPosition > 0)
    {
      enc_direction = -1;
    }

    /* maps our current outputVal from an expected range 0-100
     * to a range of 60-120 for our final motor power set */
    long outputVal = map(abs(newPosition), 0, 100, 60, 120);
    
    /* don't stall motors by running at low speeds that don't even turn the wheels*/
    if (outputVal < 10)
    {
      outputVal = 0;
    }

    /* set motor speeds */
    /* 1.2 coefficient for M1 is crude method to balance unequal motor quality */
    /* negative to reverse direction of M1 */
    md.setM1Speed(1.2*-outputVal * enc_direction);
    md.setM2Speed(outputVal * enc_direction);

    /* testing + debugging data */
    Serial.println(outputVal);
  }
}

