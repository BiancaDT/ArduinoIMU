//*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>
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


static unsigned long lastPrint = 0; // Keep track of print time

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
    float accx = -imu.calcAccel(imu.ay);
    float accy = -imu.calcAccel(imu.ax);
    float accz = -imu.calcAccel(imu.az);
    float gyrox = imu.calcGyro(imu.gy)* DEG_TO_RAD;
    float gyroy = -imu.calcGyro(imu.gx)* DEG_TO_RAD;
    float gyroz = imu.calcGyro(imu.gz)* DEG_TO_RAD;
    float magx = -imu.calcMag(imu.mx);
    float magy = -imu.calcMag(imu.my);
    float magz = imu.calcMag(imu.mz);
         
  deltat = fusion.deltatUpdate();
  
  //fusion.MahonyUpdate(gyrox, gyroy, gyroz, accx, accy, accz, magy, magx, magz, deltat);
  fusion.MadgwickUpdate(gyrox, gyroy, gyroz, accx, accy, accz, magy, magx, magz, deltat);  

  pitchQ = fusion.getPitch();
  rollQ = fusion.getRoll();    //could also use getRollRadians() ecc
  yawQ = fusion.getYaw();
 
  q0 = fusion.getQuatw();
  q1 = fusion.getQuatx();
  q2 = fusion.getQuaty();
  q3 = fusion.getQuatz();

  float roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)* RAD_TO_DEG;
  float pitch = asinf(-2.0f * (q1*q3 - q0*q2))* RAD_TO_DEG;;
  float yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)* RAD_TO_DEG+180;

  Serial.print("yay quats: ");Serial.print(q0);Serial.print(",");Serial.print(q1);Serial.print(",");Serial.print(q2);Serial.print(",");Serial.print(q3);Serial.print("!");

  Serial.print(pitchQ);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(rollQ);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(yawQ);
  Serial.print(",");
  Serial.println(yaw);
  delay(200);              
  } 
}
