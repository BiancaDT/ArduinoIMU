//*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>
#include <Servo.h>
#include "SensorFusion.h"
SF fusion;

//////////////////////////
// LSM9DS1 Library Init //

LSM9DS1 imu;
////////////////////////////
// Sketch Output Settings //
////////////////////////////

//#define PRINT_RAW
#define PRINT_CALCULATED
#define PRINT_SPEED 300 // 250 ms between prints

#define DECLINATION 4.25 // Declination (degrees) in Copenhagen, Denmark

float deltat;
float rollQ, pitchQ, yawQ;
float q0, q1, q2, q3;

Servo pitchServo;
Servo rollServo;

int servoPan = 2;
int servoTilt = 3;
static unsigned long lastPrint = 0; // Keep track of print time

float rollActual, pitchActual, yawActual;
float rollError, pitchError =0;
float rollErrorOld, pitchErrorOld, rollErrorChange, pitchErrorChange;
float rollTarget=0; //neutral position 75 for both
float rollErrorSlope, rollErrorArea=0;
float pitchErrorSlope, pitchErrorArea=0;
float pitchTarget=0;
float rollServoA=75; 
float pitchServoA=75;

////PID stuff
float k1 = .5;
float k2 = 70; //D term, operating in miliseconds, in k2 we div by dt, large nr
float k3 = .001; //I term, multiplied by dt
int miliOld, miliNew, dt;


//Function definitions
void printGyro();
void printAccel();
void printMag();

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    while (1);
  } 
  imu.calibrate(true);    
  imu.calibrateMag(true);  
}

void loop()
{
  if ( imu.gyroAvailable() )
  {
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    imu.readMag();
  }

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    float accx = -imu.calcAccel(imu.ax);
    float accy = -imu.calcAccel(imu.ay);
    float accz = imu.calcAccel(imu.az);
    float gyrox = imu.calcGyro(imu.gx)* DEG_TO_RAD;
    float gyroy = imu.calcGyro(imu.gy)* DEG_TO_RAD;
    float gyroz = imu.calcGyro(imu.gz)* DEG_TO_RAD;
    float magx = imu.calcMag(imu.my);
    float magy = imu.calcMag(imu.mx);
    float magz = imu.calcMag(imu.mz);
         
  deltat = fusion.deltatUpdate();
  
  //fusion.MahonyUpdate(gyrox, gyroy, gyroz, accx, accy, accz, magx, magy, magz, deltat);
  fusion.MadgwickUpdate(gyrox, gyroy, gyroz, accx, accy, accz, magx, magy, magz, deltat);  

  pitchQ = fusion.getPitch();
  rollQ = fusion.getRoll();    //could also use getRollRadians() ecc
  yawQ = fusion.getYaw()-4.25f;
 
  q0 = fusion.getQuatw();
  q1 = fusion.getQuatx();
  q2 = fusion.getQuaty();
  q3 = fusion.getQuatz();

  float roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)* RAD_TO_DEG;
  float pitch = asinf(-2.0f * (q1*q3 - q0*q2))* RAD_TO_DEG;;
  float yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)* RAD_TO_DEG-4.25f;

  Serial.print("yay quats: ");Serial.print(q0);Serial.print(",");Serial.print(q1);Serial.print(",");Serial.print(q2);Serial.print(",");Serial.print(q3);Serial.print("!");

  //MilliOld=milliNew;
  //milliNew=millis();
  //dt = milliNew-milliOld;

  //rollErrorOld=rollError
  //rollError=rollTarget-rollActual;
  //rollErrorChange=rollError-rollErrorOld
  //rollErrorSlope = rollErrorChange/dt;
  //rollErrorArea=rollErrorArea+rollError*dt

  //Serial.print(roll);
  //Serial.print(",");
 
  //Serial.print(pitch);
  //Serial.print(",");
  Serial.print(rollQ);
  Serial.print(",");
   Serial.print(pitchQ);
  Serial.print(",");
  Serial.println(yawQ);
  //Serial.print(",");
  //Serial.println(yaw);
  delay(200);              
  } 
}
