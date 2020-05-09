//*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>

//////////////////////////
// LSM9DS1 Library Init //

LSM9DS1 imu;
////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints


// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. 
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 4.25 // Declination (degrees) in Copenhagen, Denmark

float rollFold = 0;
float rollFnew;
float pitchFold = 0;
float pitchFnew;

float rollG = 0;
float pitchG = 0;
float dt;

float rollC, pitchC;

static unsigned long lastPrint = 0; // Keep track of print time

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printTemp();

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
  imu.calibrate(true);         //calibrates but does not store bias
  imu.calibrateMag(true);  
  lastPrint=millis();

}

void loop()
{
  if( imu.tempAvailable() ){    // Update when new sensor data is available
        imu.readTemp();           // Read the temperature
        }
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
    //printTemp();
    //printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"          
    printAttitude(imu.ay, imu.ax, imu.az,
                  -imu.mx, -imu.my, imu.mz,
                  -imu.calcGyro(imu.gy), -imu.calcGyro(imu.gx));    
    //printGyro();  // Print "G: gx, gy, gz"       
    Serial.println();
  } 
}

void printGyro()
{

#ifdef PRINT_CALCULATED
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(-imu.calcGyro(imu.gx), 2);  
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  //Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
#ifdef PRINT_CALCULATED
  Serial.print(-imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(-imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(-imu.calcAccel(imu.az), 2);
#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
  Serial.print(", ");
#endif
}

void printMag()
{// Either print them as raw ADC values, or calculated in Gauss.
  //Serial.print("M: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  //Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

void printTemp()
{
  Serial.print("Temp: ");
  Serial.println(imu.temperature);
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy)
{
  float roll = atan2(ax, az);
  float pitch = atan2(ay, sqrt(ax * ax + az * az));

  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  //Filtered acc stuff
  rollFnew = .9*rollFold + .1*roll;
  pitchFnew = .9*pitchFold + .1*pitch;

  dt=(millis()-lastPrint)/1000.; 
  lastPrint = millis(); 

  rollG = rollG + gy*dt;
  pitchG = pitchG - gx*dt;
  
  rollC = (rollC + gy*dt)*.9 + rollFnew*.1;
  pitchC = (pitchC - gx*dt)*.9 + pitchFnew*.1;

  //Conversion deg to rad
  float rollRad = rollC/360*(2*PI);
  float pitchRad = pitchC/360*(2*PI);
 
  //Projection in 3D in radians

  //mx = mx*cos(pitchRad)-my*sin(rollRad)*sin(pitchRad)+mz*cos(rollRad)*sin(pitchRad);
  //my = my*cos(rollRad)+mz*sin(rollRad);

  //my = my*cos(pitchRad)-mx*sin(rollRad)*sin(pitchRad)+mz*cos(rollRad)*sin(pitchRad);
  //mx = mx*cos(rollRad)+mz*sin(rollRad);

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(my, mx);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;

  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(rollFnew);
  Serial.print(", ");
  Serial.print(pitchFnew);
  Serial.print(", ");
  Serial.print(rollG);
  Serial.print(", ");
  Serial.print(pitchG);
  Serial.print(", ");
  Serial.print(rollC);
  Serial.print(", ");
  Serial.println(pitchC);

  rollFold = rollFnew;
  pitchFold = pitchFnew;
  
}
