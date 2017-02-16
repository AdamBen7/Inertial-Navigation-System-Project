#include <Wire.h>
#include "GY_85.h"

GY_85 GY85;
char choice;

// Raw Ranges:
// initialize to mid-range and allow calibration to
// find the minimum and maximum for each axis
double xRawMin = 0;
double xRawMax = 0;

double yRawMin = 0;
double yRawMax = 0;

double zRawMin = 0;
double zRawMax = 0;

// Take multiple samples to reduce noise
const int sampleSize = 10;

void setup() 
{
    Wire.begin();
    delay(10);
    Serial.begin(9600);
    delay(10);
    GY85.init();
    delay(10);
}

void loop() 
{
  
  double xRaw = 0;
  double yRaw = 0;
  double zRaw = 0;
  
  for(int i=0; i<100; i++){
    xRaw += GY85.accelerometer_x( GY85.readFromAccelerometer() );
    yRaw += GY85.accelerometer_y( GY85.readFromAccelerometer() );
    zRaw += GY85.accelerometer_z( GY85.readFromAccelerometer() );
  }

  xRaw /= 100.0;
  yRaw /= 100.0;
  zRaw /= 100.0;
    
  choice = 'q';

  if (Serial.available() > 0){
    Serial.print("Enter 'c' for calibrating.");
    choice = Serial.read();
  }
//choice = 'c';
  if (choice == 'c')
  {
    AutoCalibrate(xRaw, yRaw, zRaw);
  }
  else
  {
    Serial.print("Raw Ranges: X: ");
    Serial.print(xRawMin);
    Serial.print("-");
    Serial.print(xRawMax);
    
    Serial.print(", Y: ");
    Serial.print(yRawMin);
    Serial.print("-");
    Serial.print(yRawMax);
    
    Serial.print(", Z: ");
    Serial.print(zRawMin);
    Serial.print("-");
    Serial.print(zRawMax);
    Serial.println();
    Serial.print(xRaw);
    Serial.print(", ");
    Serial.print(yRaw);
    Serial.print(", ");
    Serial.print(zRaw);
      
    // Convert raw values to 'milli-Gs"
    long xScaled = map(xRaw, xRawMin, xRawMax, -1000, 1000);
    long yScaled = map(yRaw, yRawMin, yRawMax, -1000, 1000);
    long zScaled = map(zRaw, zRawMin, zRawMax, -1000, 1000);
   
    // re-scale to fractional Gs
    float xAccel = xScaled / 1000.0;
    float yAccel = yScaled / 1000.0;
    float zAccel = zScaled / 1000.0;
  
    Serial.print(" :: ");
    Serial.print(xAccel);
    Serial.print("G, ");
    Serial.print(yAccel);
    Serial.print("G, ");
    Serial.print(zAccel);
    Serial.println("G");
   
    
  } 
  delay(500);
}
//
// Read "sampleSize" samples and report the average
//
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading/sampleSize;
}

//
// Find the extreme raw readings from each axis
//
void AutoCalibrate(double xRaw, double yRaw, double zRaw)
{
  Serial.println("Calibrate");
  
  if (xRaw < xRawMin)
  {
    xRawMin = xRaw;
  }
  if (xRaw > xRawMax)
  {
    xRawMax = xRaw;
  }
  
  if (yRaw < yRawMin)
  {
    yRawMin = yRaw;
  }
  if (yRaw > yRawMax)
  {
    yRawMax = yRaw;
  }

  if (zRaw < zRawMin)
  {
    zRawMin = zRaw;
  }
  if (zRaw > zRawMax)
  {
    zRawMax = zRaw;
  }
}
