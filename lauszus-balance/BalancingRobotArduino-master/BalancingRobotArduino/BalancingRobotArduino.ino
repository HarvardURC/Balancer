/*
 * The code is released under the GNU General Public License.
 * Developed by Kristian Lauszus
 * This is the algorithm for my balancing robot/segway.
 * It is controlled by either an Android app or a Processing application via bluetooth.
 * The Android app can be found at the following link: https://github.com/TKJElectronics/BalanduinoAndroidApp
 * The Processing application can be found here: https://github.com/TKJElectronics/BalancingRobotArduino/tree/master/ProcessingApp
 * The SPP Bluetooth Library can be found at the following link: https://github.com/felis/USB_Host_Shield_2.0
 * For details, see http://blog.tkjelectronics.dk/2012/02/the-balancing-robot/
 */

#include "BalancingRobot.h"
#include <Kalman.h> // Kalman filter library see: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
Kalman kalman; // See https://github.com/TKJElectronics/KalmanFilter for source code
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SSD1306_text.h>
#include "DualVNH5019MotorShield.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (10) //maybe lower this
Adafruit_BNO055 bno = Adafruit_BNO055();

#define OLED_RESET 1
SSD1306_text display(OLED_RESET);


DualVNH5019MotorShield md;


#define UP_DOWN_IN_PIN   3  //maybe flip these 2
#define  LEFT_RIGHT_IN_PIN  5

//#include <SPP.h> // SS is rerouted to 8 and INT is rerouted to 7 - see http://www.circuitsathome.com/usb-host-shield-hardware-manual at "5. Interface modifications"
//USB Usb;
//BTD Btd(&Usb); // Uncomment DEBUG in "BTD.cpp" to save space
//SPP SerialBT(&Btd,"BalancingRobot","0000"); // Also uncomment DEBUG in "SPP.cpp"
 double accX;
  double accY;
  double accZ;
  double gyroY;
  int speed_1; // made global
void setup() {
  /* Setup encoders */
    Serial.begin(9600);
  Wire.begin();
    display.init();
    display.clear();
    delay(200);
     display.setTextSize(2,1);
    display.setCursor(4,40);
    display.print("test");
    delay(1000);
    
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT); 
  attachInterrupt(leftEncoder1,leftEncoder,CHANGE); // pin 2   //TODO both was RISING
  attachInterrupt(rightEncoder1,rightEncoder,CHANGE); // pin 3
  delay(200);
 // Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
 // Serial.println("BNO good"); Serial.println("");

  delay(200); // Wait for sensor to stabilize
    bno.setExtCrystalUse(true);
    
    imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  double accX = accelerometer.x();
  double accY = accelerometer.y();
  double accZ = accelerometer.z();
  /* Setup motor pins to output */
  /*sbi(leftPwmPortDirection,leftPWM);
  sbi(leftPortDirection,leftA);
  sbi(leftPortDirection,leftB);
  sbi(rightPwmPortDirection,rightPWM);
  sbi(rightPortDirection,rightA);
  sbi(rightPortDirection,rightB);  
  */
  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8025.pdf page 128-135 */
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  //TCCR1B = _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling  I don't think these 3 are neccessary
  //ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  //ICR1L = (PWMVALUE & 0xFF);

  /* Enable PWM on pin 9 (OC1A) & pin 10 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when downcountin
 // TCCR1A = _BV(COM1A1) | _BV(COM1B1);                                againidk if we need this TODO
 // setPWM(leftPWM,0); // Turn off pwm on both pins
 // setPWM(rightPWM,0);

  /* Setup pin for buzzer to beep when finished calibrating */
 // pinMode(buzzer,OUTPUT);  

  /* Setup IMU Inputs */
 // pinMode(gyroY,INPUT);
  //pinMode(accY,INPUT);
  //pinMode(accZ,INPUT);      

  /* Calibrate the gyro and accelerometer relative to ground */
 // calibrateSensors();   TODO maybe add calibration??
  /* Setup timing */
  loopStartTime = micros();
  timer = loopStartTime;
}

void loop() {
  /* Calculate pitch */    
   // imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   //imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  accYangle = getAccY();
  gyroYrate = getGyroYrate();
  gyroAngle += gyroYrate*((double)(micros()-timer)/1000000);
  // See my guide for more info about calculation the angles and the Kalman filter: http://arduino.cc/forum/index.php/topic,58048.0.htm
  pitch = euler.y()+180; //accYangle;//kalman.getAngle(accYangle, gyroYrate, (double)(micros() - timer)/1000000); // Calculate the angle using a Kalman filter
  timer = micros();  
  Serial.print("pitch: "); Serial.println(pitch);
  /* Drive motors */
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if((layingDown && (pitch < 170 || pitch > 190)) || (!layingDown && (pitch < 135 || pitch > 225))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } 
  else {
    layingDown = false; // It's no longer laying down
    PID2(targetAngle,targetOffset,turningOffset);        
  }

  /* Update wheel velocity every 100ms */
  loopCounter++;
  if (loopCounter == 10) {
    loopCounter = 0; // Reset loop counter
    wheelPosition = readLeftEncoder() + readRightEncoder();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    if (abs(wheelVelocity) <= 20 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
  /* Read the SPP connection */
 // readSPP();    
 /* if(SerialBT.connected) {
    Usb.Task();
    if(sendPIDValues) {
      sendPIDValues = false;
      strcpy(stringBuf,"P,");
      strcat(stringBuf,SerialBT.doubleToString(Kp,2));
      strcat(stringBuf,",");
      strcat(stringBuf,SerialBT.doubleToString(Ki,2));
      strcat(stringBuf,",");
      strcat(stringBuf,SerialBT.doubleToString(Kd,2));
      strcat(stringBuf,",");
      strcat(stringBuf,SerialBT.doubleToString(targetAngle,2));
      SerialBT.println(stringBuf);
      dataCounter = 1;
    } else if(sendData) {
      switch(dataCounter) {
        case 0:      
          strcpy(stringBuf,"V,");
          strcat(stringBuf,SerialBT.doubleToString(accYangle,2));
          strcat(stringBuf,",");
          strcat(stringBuf,SerialBT.doubleToString(gyroAngle,2));
          strcat(stringBuf,",");
          strcat(stringBuf,SerialBT.doubleToString(pitch,2));
          SerialBT.println(stringBuf);
          break;
      }    
      dataCounter++;
      if(dataCounter > 4)
        dataCounter = 0;    
    }
  } */

  /* Use a time fixed loop */
  lastLoopUsefulTime = micros() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    while((micros() - loopStartTime) < STD_LOOP_TIME)
    {
      //  Usb.Task();  read wireless here???
    }
      
  }
  loopStartTime = micros();    
}
void PID2(double restAngle, double offset, double turning) {
  /* Steer robot */
  if (steerForward) {
    offset += (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed and scale up when reversing
    restAngle -= offset;
  } 
  else if (steerBackward) {
    offset -= (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed and scale up when reversing
    restAngle += offset;
  }
  /* Brake */
  else if (steerStop) {
    long positionError = wheelPosition - targetPosition;
    if (abs(positionError) > zoneA) // Inside zone A
      restAngle -= (double)positionError/positionScaleA;
    else if (abs(positionError) > zoneB) // Inside zone B
      restAngle -= (double)positionError/positionScaleB;
    else // Inside zone C
      restAngle -= (double)positionError/positionScaleC;   
    restAngle -= (double)wheelVelocity/velocityScaleStop;
    if (restAngle < 160) // Limit rest Angle
      restAngle = 160;
    else if (restAngle > 200)
      restAngle = 200;
  }
  /* Update PID values */
  double error = (restAngle - pitch);
  double pTerm = Kp * error;
  iTerm += Ki * error;
  double dTerm = Kd * (error - lastError);
  lastError = error;
  double PIDValue = pTerm + iTerm + dTerm;
  //Serial.print("PIDValue"); Serial.println(PIDValue);
  /* Steer robot sideways */
  double PIDLeft;
  double PIDRight;
  if (steerLeft) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if (steerRight) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }

  PIDLeft *= 0.95; // compensate for difference in the motors     TODO check this

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, PIDLeft * -1);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, PIDRight * -1);
}
/*
void readSPP() {
  //Usb.Task();
  if(SerialBT.connected) {
    if(SerialBT.available()) {
      char input[30];
      uint8_t i = 0;
      while (1) {
        input[i] = SerialBT.read();
        if(input[i] == -1) // Error while reading the string
          return;
        if (input[i] == ';') // Keep reading until it reads a semicolon
          break;
        i++;
      }      
      Serial.print("Data: ");
      Serial.write((uint8_t*)input,i);
      Serial.println();
      if(input[0] == 'A') { // Abort
        stopAndReset();
       // while(SerialBT.read() != 'C') // Wait until continue is send
          //Usb.Task();
      } 
      
      // Set PID and target angle 
      else if(input[0] == 'P') {
        strtok(input, ","); // Ignore 'P'
        Kp = atof(strtok(NULL, ";"));
      } else if(input[0] == 'I') {
        strtok(input, ","); // Ignore 'I'
        Ki = atof(strtok(NULL, ";"));  
      } else if(input[0] == 'D') {
        strtok(input, ","); // Ignore 'D'
        Kd = atof(strtok(NULL, ";"));  
      } else if(input[0] == 'T') { // Target Angle
        strtok(input, ","); // Ignore 'T'
        targetAngle = atof(strtok(NULL, ";"));  
      } else if(input[0] == 'G') { // The processing/Android application sends when it need the current values
        if(input[1] == 'P') // PID Values
          sendPIDValues = true;
        else if(input[1] == 'B') // Begin
          sendData = true; // Send output to processing/Android application
        else if(input[1] == 'S') // Stop
          sendData = false; // Stop sending output to processing/Android application
      }
      // Remote control 
      else if(input[0] == 'S') // Stop
        steer(stop);      
      else if(input[0] == 'J') { // Joystick
        strtok(input, ","); // Ignore 'J'
        sppData1 = atof(strtok(NULL, ",")); // x-axis
        sppData2 = atof(strtok(NULL, ";")); // y-axis
        steer(joystick);
      }
      else if(input[0] == 'M') { // IMU
        strtok(input, ","); // Ignore 'I'
        sppData1 = atof(strtok(NULL, ",")); // Pitch
        sppData2 = atof(strtok(NULL, ";")); // Roll
        steer(imu);
        //SerialBT.printNumberln(sppData1);
        //SerialBT.printNumberln(sppData2);
      }
    }
  } else
    steer(stop);
}
*/
/*
void steer(Command command) {
  // Set all false
  steerForward = false;
  steerBackward = false;
  steerStop = false;
  steerLeft = false;
  steerRight = false;
  if(command == joystick) {    
    if(sppData2 > 0) {
      targetOffset = scale2(sppData2,0,1,0,7);        
      steerForward = true;
    } else if(sppData2 < 0) {
      targetOffset = scale2(sppData2,0,-1,0,7);
      steerBackward = true;
    } 
    if(sppData1 > 0) {
      turningOffset = scale2(sppData1,0,1,0,20);        
      steerRight = true;
    } else if(sppData1 < 0) {
      turningOffset = scale2(sppData1,0,-1,0,20);
      steerLeft = true;     
    }
  } else if(command == imu2) {
      if(sppData2 > 0) {
        targetOffset = scale2(sppData2,1,36,0,7);        
        steerForward = true;
      }     
      else if(sppData2 < 0) {
        targetOffset = scale2(sppData2,-1,-36,0,7);
        steerBackward = true;
      }
      if(sppData1 > 0) {
        turningOffset = scale2(sppData1,1,45,0,20);        
        steerLeft = true;
      }
      else if(sppData1 < 0) {
        turningOffset = scale2(sppData1,-1,-45,0,20);
        steerRight = true;     
      }
  }
  
  else if(command == stop) {
    steerStop = true;    
    if(lastCommand != stop) { // Set new stop position
      targetPosition = wheelPosition;
      stopped = false;
    }
  }
  lastCommand = command;
} */
double scale2(double input, double inputMin, double inputMax, double outputMin, double outputMax) { // Like map() just returns a double
  double output;
  if(inputMin < inputMax)
    output = (input-inputMin)/((inputMax-inputMin)/(outputMax-outputMin));              
  else
    output = (inputMin-input)/((inputMin-inputMax)/(outputMax-outputMin));
  if(output > outputMax)
    output = outputMax;
  else if(output < outputMin)
    output = outputMin;
  return output;
}
void stopAndReset() {
  stopMotor(left);
  stopMotor(right);  
  lastError = 0;
  iTerm = 0;
  targetPosition = wheelPosition;
}
double getGyroYrate() {
   imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
   gyroY = gyroscope.y();
  // (gyroAdc-gyroZero)/Sensitivity (In quids) - Sensitivity = 0.00333/3.3*1023=1.0323
  double gyroRate = -((double)((double)(gyroY) - zeroValues[0]) / 1.0323);
  return gyroRate;
}
double getAccY() {
     imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  accX = accelerometer.x(); 
  accY = accelerometer.y();
  accZ = accelerometer.z();
  double accXval = ((double)(accX) - zeroValues[0]);
  double accYval = ((double)(accY) - zeroValues[1]);  
  double accZval = ((double)(accZ) - zeroValues[2]);
  // Convert to 360 degrees resolution
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We are then convert it to 0 to 2π and then from radians to degrees
  return (atan2(-accXval,accZval)+PI)*RAD_TO_DEG;
}
void calibrateSensors() {
  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
    zeroValues[0] += (gyroY);
    zeroValues[1] += (accY);
    zeroValues[2] += (accZ);
    delay(10);
  }
  zeroValues[0] /= 100; // Gyro X-axis
  zeroValues[1] /= 100; // Accelerometer Y-axis
  zeroValues[2] /= 100; // Accelerometer Z-axis

  if(zeroValues[1] > 500) { // Check which side is lying down - 1g is equal to 0.33V or 102.3 quids (0.33/3.3*1023=102.3)
    zeroValues[1] -= 102.3; // +1g when lying at one of the sides
    kalman.setAngle(90); // It starts at 90 degress and 270 when facing the other way
    gyroAngle = 90;
  } else {
    zeroValues[1] += 102.3; // -1g when lying at the other side
    kalman.setAngle(270);
    gyroAngle = 270;
  }
}
void moveMotor(Command motor, Command direction, double speedRaw) { // Speed is a value in percentage 0-100%
  if(speedRaw > 100)
    speedRaw = 100;
   speed_1 = speedRaw*((double)PWMVALUE)/100; // Scale from 100 to `///
   
  if (motor == left) {
   // setPWM(leftPWM,speed_1); // Left motor pwm
    if (direction == forward) {
       md.setM1Speed(speed_1);
      //cbi(leftPort,leftA);
      //sbi(leftPort,leftB);
    } 
    else if (direction == backward) {
      md.setM1Speed(-speed_1);
     // sbi(leftPort,leftA);
     // cbi(leftPort,leftB);
    }
  } 
  else if (motor == right) {
   // setPWM(rightPWM,speed); // Right motor pwm
    if (direction == forward) {
      md.setM2Speed(speed_1);
     // cbi(rightPort,rightA);
     // sbi(rightPort,rightB);
    } 
    else if (direction == backward) {
       md.setM2Speed(-speed_1);
      //sbi(rightPort,rightA);
      //cbi(rightPort,rightB);
    }
  }
}
void stopMotor(Command motor) {  
  if (motor == left){
     md.setM1Brake(400);  
  } 
  else if (motor == right) {
     md.setM2Brake(400);              //TODO flip these two
  }
}


/* Interrupt routine and encoder read functions - I read using the port registers for faster processing */
void leftEncoder() { 
  if(digitalRead(A3) == HIGH) // read pin 4
    leftCounter--;
  else
    leftCounter++;    
}
void rightEncoder() {
  if(digitalRead(A1) == HIGH)   //PIND & _BV(PIND5)) // read pin 5      TODO make this faster by bitshift
    rightCounter--;
  else
    rightCounter++;  
}
long readLeftEncoder() { // The encoders decrease when motors are traveling forward and increase when traveling backward
  return leftCounter;
}
long readRightEncoder() {
  return rightCounter;
}
