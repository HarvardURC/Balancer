// MPU-6050
// CREDIT: JohnChi
// Public Domain
#include<Wire.h>
#define pi 3.14159 
#define Angle_offset 2
#define Gry_offset -1 
#define Gyr_Gain 0.00763358  
#define RMotor_offset 100
#define LMotor_offset 100
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
float kp, ki, kd; 
unsigned long preTime = 0;
float SampleTime = 0.08;
unsigned long lastTime;
float r_angle, f_angle, omega;
int timeChange; 
float Input, Output;
float errSum, dErr, error, lastErr;
float LOutput,ROutput;
float Run_Speed = 0, Run_Speed_K = 0, Run_Speed_T = 0;
float Turn_Speed = 0, Turn_Speed_K = 0, outputl =0, outputr =0;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  kp = 41;
  kd =0;//1.4; //was
  ki = 0;
  md.init();
  md.setM1Speed(0);
  md.setM2Speed(0);
}
void loop(){
  //md.setM1Speed(250);
  unsigned long now = millis();
  float dt = (now - preTime) / 1000.0;
  preTime = now;
  float K = 0.8;
  float A = K / (K + dt);
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  float r_angle = (atan2(AcY, AcZ) * 180 / pi + Angle_offset);
  float omega =  Gyr_Gain * (GyX) + Gry_offset; 
  f_angle = A * (f_angle + omega * dt) + (1 - A) * r_angle; 
  timeChange = (now - lastTime);
  if(timeChange >= SampleTime){
    Input = f_angle;
    error = Input;
    errSum += error * timeChange;
    dErr = (error - lastErr) / timeChange;
    Output = kp * error + ki * errSum + kd * dErr;
    LOutput = Output + Run_Speed + Turn_Speed;  Serial.print("  LOutput=");Serial.print(outputl);
    ROutput = Output + Run_Speed - Turn_Speed;  Serial.print("  ROutput=");Serial.println(outputr);
    lastErr = error;
    lastTime = now;
  }
  
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | omega = "); Serial.print(omega);
  Serial.print(" sample"); Serial.print(SampleTime);
  Serial.print(" timeChange"); Serial.print(timeChange);
  Serial.print(" | r_angle = "); Serial.print(r_angle);
  outputl = map((LOutput+LMotor_offset), -1024,1023,-400,400);
  outputr = map((ROutput+RMotor_offset), -1024,1023,400,-400);
  md.setM2Speed(outputl);
  md.setM1Speed(outputr);
  if(abs(r_angle)>42)
  {
     md.setM2Speed(0);
  md.setM1Speed(0);
  }
 
}
