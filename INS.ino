#include "Quadcopter.h"
#include <Wire.h>
#include "GY_85.h"

using namespace std;

void LinearUpdater(Quadcopter &);
void RotationUpdater(Quadcopter &);
void InitGravity(Quadcopter &);


Quadcopter MyQuad;
GY_85 GY85;
double grav;

void setup()
{
  Wire.begin();
  delay(10);
  Serial.begin(9600);
  delay(10);
  Serial.print("Initializing...");
  GY85.init();
  InitGravity(MyQuad);
  delay(10);
  MyQuad.SetTime((double)(millis()) / 1000.0);
}

int i = 0;

void loop()
{
  for (int i = 0; i < 10; i++)
  {
    MyQuad.SetTime((double)(millis()) / 1000.0);
    //RotationUpdater(MyQuad);
    LinearUpdater(MyQuad);
  }
  DisplayState();
}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++    Rotation Updater    ++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RotationUpdater(Quadcopter & SomeQuad)
{
  double dtime = SomeQuad.GetDTime();

  double * AngVelVec;
  AngVelVec = GetAngularVelocity();

  double q0 = SomeQuad.Getq0();
  double q1 = SomeQuad.Getq1();
  double q2 = SomeQuad.Getq2();
  double q3 = SomeQuad.Getq3();

  double p = SomeQuad.GetPAngVel();
  double q = SomeQuad.GetQAngVel();
  double r = SomeQuad.GetRAngVel();

  double dq0 = (p * q1 + q * q2 + r * q3) / (-2.0);
  double dq1 = ((-1 * p * q0) + (-1 * r * q2) + (q * q3)) / (-2.0);
  double dq2 = ((-1 * q * q0) + (r * q1) + (-1 * p * q3)) / (-2.0);
  double dq3 = ((-1 * r * q0) + (-1 * q * q1) + (p * q2)) / (-2.0);

  SomeQuad.Setq0(q0 + dq0 * dtime);
  SomeQuad.Setq1(q1 + dq1 * dtime);
  SomeQuad.Setq2(q2 + dq2 * dtime);
  SomeQuad.Setq3(q3 + dq3 * dtime);

  SomeQuad.NormalizeQuat();

  SomeQuad.SetPAngVel(AngVelVec[0]);
  SomeQuad.SetQAngVel(AngVelVec[1]);
  SomeQuad.SetRAngVel(AngVelVec[2]);

  delete[] AngVelVec;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++    Linear Updater    ++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void LinearUpdater(Quadcopter & SomeQuad) {

  double dtime = SomeQuad.GetDTime();
  double * AccelVec;
  AccelVec = GetAcceleration();

  double q0 = SomeQuad.Getq0();
  double q1 = SomeQuad.Getq1();
  double q2 = SomeQuad.Getq2();
  double q3 = SomeQuad.Getq3();

  double xPos = SomeQuad.GetXPos();
  double yPos = SomeQuad.GetYPos();
  double zPos = SomeQuad.GetZPos();

  //bodyframe linear vel
  double u = SomeQuad.GetUVel();
  double v = SomeQuad.GetVVel();
  double w = SomeQuad.GetWVel();

  double LVelVec[3] = {u, v, w};

  //Angular Velocity, Body Frame
  double p = SomeQuad.GetPAngVel();
  double q = SomeQuad.GetQAngVel();
  double r = SomeQuad.GetRAngVel();

  double BMat[][3] =
  { {q0*q0 + q1*q1 - q2*q2 - q3 * q3, 2 * (q1 * q2 - q0 * q3),         2 * (q1 * q3 + q0 * q2)},
    {2.0 * (q1 * q2 + q0 * q3),        q0*q0 - q1*q1 + q2*q2 - q3 * q3, 2.0 * (q2 * q3 - q0 * q1)},
    {2.0 * (q1 * q3 - q0 * q2),        2.0 * (q2 * q3 + q0 * q1),       q0*q0 - q1*q1 - q2*q2 + q3 * q3}
  };

  double xAccel = BMat[0][0] * AccelVec[0] + BMat[0][1] * AccelVec[1] + BMat[0][2] * AccelVec[2] ;
  double yAccel = BMat[1][0] * AccelVec[0] + BMat[1][1] * AccelVec[1] + BMat[1][2] * AccelVec[2] ;
  double zAccel = BMat[2][0] * AccelVec[0] + BMat[2][1] * AccelVec[1] + BMat[2][2] * AccelVec[2] ;

  double dxVel = xAccel - SomeQuad.GetGravityX(); //xAccel - Gravity
  double dyVel = yAccel - SomeQuad.GetGravityY(); //yAccel
  double dzVel = zAccel - SomeQuad.GetGravityZ(); //zAccel


  double dxPos = SomeQuad.GetdXPos() + (dxVel * dtime);
  double dyPos = SomeQuad.GetdYPos() + (dyVel * dtime);
  double dzPos = SomeQuad.GetdZPos() + (dzVel * dtime);

  SomeQuad.SetdXPos(dxPos);
  SomeQuad.SetdYPos(dyPos);
  SomeQuad.SetdZPos(dzPos);

  //derivative of x,y,z
  double dPosVector[3] = {dxPos, dyPos, dzPos};

  //equations of motion
  
  double du = r * v - q * w  + AccelVec[0] - SomeQuad.GetGravityX();
  double dv = -r * u + p * w + AccelVec[1] - SomeQuad.GetGravityY();
  double dw = q * u - p * v  + AccelVec[2] - SomeQuad.GetGravityZ();

  // Update Quadcopter Object
  SomeQuad.SetXPos(xPos + dPosVector[0]*dtime);
  SomeQuad.SetYPos(yPos + dPosVector[1]*dtime);
  SomeQuad.SetZPos(zPos + dPosVector[2]*dtime);

  SomeQuad.SetUVel(u + du * dtime);
  SomeQuad.SetVVel(v + dv * dtime);
  SomeQuad.SetWVel(w + dw * dtime);

  delete[] AccelVec;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++    STATE VECTORS    ++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

double* GetAcceleration() {
  double * AccelVec = new double [3];

  double ax = GY85.accelerometer_x(GY85.readFromAccelerometer());
  double ay = GY85.accelerometer_y(GY85.readFromAccelerometer());
  double az = GY85.accelerometer_z(GY85.readFromAccelerometer());

      // Convert raw values to 'milli-Gs"
  long xScaled = map(ax, -255.62, 264.59, -1000, 1000);
  long yScaled = map(ay, -248.07, 269.48, -1000, 1000);
  long zScaled = map(az, -248.7, 248.56, -1000, 1000);
   
    // re-scale to m/s^2
  double xAccel = xScaled * .00981;
  double yAccel = yScaled * .00981;
  double zAccel = zScaled * .00981;

  AccelVec[0] = xAccel;
  AccelVec[1] = yAccel;
  AccelVec[2] = zAccel;

  return AccelVec;
}

double * GetAngularVelocity() {
  double * AngVelVec = new double [3];

  double gx = GY85.gyro_x(GY85.readGyro());
  double gy = GY85.gyro_y(GY85.readGyro());
  double gz = GY85.gyro_z(GY85.readGyro());

  AngVelVec[0] = gx * .017453; //convert from deg/s to rad/s
  AngVelVec[1] = gy * .017453;
  AngVelVec[2] = gz * .017453;

  return AngVelVec;
}

double * GetCompassReadings() {
  double * CompassVec = new double [3];

  double cx = GY85.compass_x(GY85.readFromCompass());
  double cy = GY85.compass_y(GY85.readFromCompass());
  double cz = GY85.compass_z(GY85.readFromCompass());

  CompassVec[0] = cx; //convert data.. have to figure out what kind to what. cx * .92
  CompassVec[1] = cy;
  CompassVec[2] = cz;

  return CompassVec;
}

void InitGravity(Quadcopter & SomeQuad) { 
  double * GravityVec = new double [3];
  double * AccelVec = new double [3];
  double range = 1000.0;
  
  for (int i = 0; i < 1000; i++){
    AccelVec = GetAcceleration();
    GravityVec[0] += AccelVec[0];
    GravityVec[1] += AccelVec[1];
    GravityVec[2] += AccelVec[2];   
  }
    
  GravityVec[0]/=range;
  GravityVec[1]/=range;
  GravityVec[2]/=range;

  SomeQuad.SetGravityX(GravityVec[0]);
  SomeQuad.SetGravityY(GravityVec[1]);
  SomeQuad.SetGravityZ(GravityVec[2]);   
    
  delay(10);
  delay(10);
  delete[] GravityVec;
  delete[] AccelVec;
}



void DisplayState()
{
  
    //Serial.print("xPos : ");
    //Serial.print('\t');
    Serial.print((double)millis()/1000.0);
    
    Serial.print('\t');    
    Serial.print(MyQuad.GetXPos() );
    //Serial.print("\tyPos : ");
    Serial.print('\t');
    Serial.print(MyQuad.GetYPos() );
    //Serial.print("\tzPos : ");
    Serial.print('\t');
    Serial.print(MyQuad.GetZPos() );
    //Serial.print("\tuVel : ");

    Serial.print('\t');
    Serial.print(MyQuad.GetdXPos() );
    Serial.print('\t');
    Serial.print(MyQuad.GetdYPos() );
    Serial.print('\t');
    Serial.print(MyQuad.GetdZPos() );
/*    Serial.print(MyQuad.GetUVel() );
    //Serial.print("\tvVel : ");
    Serial.print('\t');
    Serial.print(MyQuad.GetVVel() );
    //Serial.print("\twVel : ");
    Serial.print('\t');
    Serial.print(MyQuad.GetWVel() ); */
    
    Serial.print('\t');
    Serial.print(MyQuad.GetPAngVel() );
    //Serial.print("\tQAngV: ");
    Serial.print('\t');
    Serial.print(MyQuad.GetQAngVel() );
    //Serial.print("\tRAngV: ");
    Serial.print('\t');
    Serial.print(MyQuad.GetRAngVel() );
    //Serial.print("\tQuat0: ");
    Serial.print('\t');
    Serial.print(MyQuad.Getq0() );
    //Serial.print("\tQuat1: ");
    Serial.print('\t');
    Serial.print(MyQuad.Getq1() );
    //Serial.print("\tQuat2: ");
    Serial.print('\t');
    Serial.print(MyQuad.Getq2() );
    //Serial.print("\tQuat3: ");
    Serial.print('\t');
    Serial.print(MyQuad.Getq3() );

    Serial.print('\n');
}


