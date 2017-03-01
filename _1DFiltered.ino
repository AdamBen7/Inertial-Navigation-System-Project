#include "Quadcopter.h"
#include <Wire.h>
#include "GY_85.h"

using namespace std;

void LinearUpdater(Quadcopter &);
void RotationUpdater(Quadcopter &);
void InitGravity(Quadcopter &);


Quadcopter MyQuad;
GY_85 GY85;


void setup()
{
  Wire.begin();
  delay(10);
  Serial.begin(9600);
  delay(10);
  Serial.print("Initializing...");
  GY85.init();
  delay(10);
  MyQuad.SetTime((double)(millis()) / 1000.0);
}


void loop()
{
  MyQuad.SetTime((double)(millis()) / 1000.0);
  //double ax = GY85.accelerometer_x(GY85.readFromAccelerometer());
  //double a = map(ax, -255.62, 264.59, -1000.0, 1000.0);
  double a = 1;
  double dtime = MyQuad.GetDTime();
  double v = dtime*a + MyQuad.GetUVel();
  MyQuad.SetUVel(v);
  double x = dtime*v + MyQuad.GetXPos();
  MyQuad.SetXPos(x);
  Serial.print(millis());
  Serial.print('\t');
  Serial.print(x);
  Serial.print('\n');
  delay(10);
}





