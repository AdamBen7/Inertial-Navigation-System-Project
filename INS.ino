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
  GY85.init();
  InitGravity(MyQuad);
  //grav = InitializeGravity();
  delay(10);
  
}

int i = 0;

void loop()
{
  for (int i = 0; i < 10; i++)
  {
    MyQuad.SetTime((double)(millis()) / 1000.0);
    RotationUpdater(MyQuad);
    LinearUpdater(MyQuad);
  }
  DisplayState();
}

void LinearUpdater(Quadcopter & SomeQuad) {

  double mass = 10.0;
  double weight = mass * grav;
  double dtime = SomeQuad.GetDTime();
  //I'm past patiently waitin', I'm passionately smashin' every expectation, every action's an act of creation
  double * AccelVec;
  AccelVec = GetAcceleration();

  double xAccel = AccelVec[0] - SomeQuad.GetGravityX(); //xAccel - Gravity
  double yAccel = AccelVec[1] - SomeQuad.GetGravityY(); //yAccel
  double zAccel = AccelVec[2] - SomeQuad.GetGravityZ(); //zAccel

/*      Serial.print('\n');
  Serial.print((double)millis() / 1000.0);
      Serial.print('\t');
  Serial.print(xAccel);
      Serial.print('\t');
  Serial.print(yAccel);
      Serial.print('\t');
  Serial.print(zAccel);
      Serial.print('\t');

      Serial.print('\t');
  Serial.print(SomeQuad.GetGravityX());
      Serial.print('\t');
  Serial.print(SomeQuad.GetGravityY());
      Serial.print('\t');
  Serial.print(SomeQuad.GetGravityZ());
      Serial.print('\t');*/

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

  //LinearForces Experienced
  double X = 0.0;
  double Y = 0.0;
  double Z = 0.0;

  double BMat[][3] =
  { {q0*q0 + q1*q1 - q2*q2 - q3 * q3, 2 * (q1 * q2 - q0 * q3),         2 * (q1 * q3 + q0 * q2)},
    {2.0 * (q1 * q2 + q0 * q3),        q0*q0 - q1*q1 + q2*q2 - q3 * q3, 2.0 * (q2 * q3 - q0 * q1)},
    {2.0 * (q1 * q3 - q0 * q2),        2.0 * (q2 * q3 + q0 * q1),       q0*q0 - q1*q1 - q2*q2 + q3 * q3}
  };

  double dxPos = BMat[0][0] * LVelVec[0] + BMat[0][1] * LVelVec[1] + BMat[0][2] * LVelVec[2] ;
  double dyPos = BMat[1][0] * LVelVec[0] + BMat[1][1] * LVelVec[1] + BMat[1][2] * LVelVec[2] ;
  double dzPos = BMat[2][0] * LVelVec[0] + BMat[2][1] * LVelVec[1] + BMat[2][2] * LVelVec[2] ;

  //derivative of x,y,z
  double dPosVector[3] = {dxPos, dyPos, dzPos};

  //equations of motion

  double cosTheta = cos(MyQuad.GetTheta());
  double phi = MyQuad.GetPhi();
/*
  double du = r * v - q * w + xAccel;
  double dv = -r * u + p * w + yAccel;
  double dw = q * u - p * v  + zAccel;
*/
  // For tilting purposes... figure it out
  
  double du = r * v - q * w + AccelVec[0] - (grav * 2 * (q0 * q2 - q1 * q3));
  double dv = -r * u + p * w + AccelVec[1] + (grav * cosTheta * sin(phi));
  double dw = q * u - p * v  + AccelVec[2] + (grav * cosTheta * cos(phi));
  

  /* //Commented out atm since we might not need to touch euler angles.
    double du = r*v -q*w + (X/mass) - grav*sin(eulerX);
    double dv = -r*u+p*q + (Y/mass) - grav*cos(eulerX)*sin(eulerY);
    double dw = q*u-p*v  + (Z/mass) - grav*cos(eulerX)*cos(eulerY);

    //acceleration in body frame
    double dVelVector[3] = {du, dv, dw};
  */

  // Update Quadcopter Object
  SomeQuad.SetXPos(xPos + dPosVector[0]*dtime);
  SomeQuad.SetYPos(yPos + dPosVector[1]*dtime);
  SomeQuad.SetZPos(zPos + dPosVector[2]*dtime);

  SomeQuad.SetUVel(u + du * dtime);
  SomeQuad.SetVVel(v + dv * dtime);
  SomeQuad.SetWVel(w + dw * dtime);

  delete[] AccelVec;
}

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

  SomeQuad.SetPAngVel(AngVelVec[0]);
  SomeQuad.SetQAngVel(AngVelVec[1]);
  SomeQuad.SetRAngVel(AngVelVec[2]);

  delete[] AngVelVec;
}

double* GetAcceleration() {
  double * AccelVec = new double [3];

  double ax = GY85.accelerometer_x(GY85.readFromAccelerometer());
  double ay = GY85.accelerometer_y(GY85.readFromAccelerometer());
  double az = GY85.accelerometer_z(GY85.readFromAccelerometer());

  AccelVec[0] = ax * .03832; // 11-bit resolution (+-1024) to +-4g
  AccelVec[1] = ay * .03832;
  AccelVec[2] = az * .03832;

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
  double range = 80.0;
  
  for (int i = 0; i < 81; i++){
    AccelVec = GetAcceleration();
    GravityVec[0] += AccelVec[0];
    GravityVec[1] += AccelVec[1];
    GravityVec[2] += AccelVec[2];   
    if(i == range)
    {
      GravityVec[0]/=range;
      GravityVec[1]/=range;
      GravityVec[2]/=range;

      //GravityVec[1]-= 1; //calibrating 
      
      /*
    Serial.print('\n');
    Serial.print(GravityVec[0]);
    Serial.print('\t');
    Serial.print(GravityVec[1]);
    Serial.print('\t');
    Serial.print(GravityVec[2]);
    Serial.print('\n');
    */
      SomeQuad.SetGravityX(GravityVec[0]);
      SomeQuad.SetGravityY(GravityVec[1]);
      SomeQuad.SetGravityZ(GravityVec[2]);   
    }
    delay(10);
  }
/*
  double magnitude = sqrt(AccelVec[0]*AccelVec[0] + AccelVec[1]*AccelVec[1] + AccelVec[2]*AccelVec[2]);
  AccelVec[0] /= magnitude;
  AccelVec[1] /= magnitude;
  AccelVec[2] /= magnitude;

    if (AccelVec[3] >= 0.0)
  {
    SomeQuad.Setq0(sqrt((AccelVec[2] + 1.0) / 2.0));
    SomeQuad.Setq1(-1.0 * AccelVec[1] / sqrt(2.0 * (AccelVec[2] + 1.0)));
    SomeQuad.Setq2(AccelVec[0] /sqrt(2.0 * (AccelVec[2] + 1.0)));
    SomeQuad.Setq3(0.0);
  }

  if (AccelVec[3] < 0.0)
  {
    SomeQuad.Setq0(-1.0 * AccelVec[1] /sqrt(2.0 * (1.0 - AccelVec[2])));
    SomeQuad.Setq1(sqrt((1.0 - AccelVec[2]) / 2.0));
    SomeQuad.Setq2(0.0);
    SomeQuad.Setq3(AccelVec[0] / sqrt(2.0 * (1.0 - AccelVec[2])));
*/
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
    Serial.print(MyQuad.GetUVel() );
    //Serial.print("\tvVel : ");
    Serial.print('\t');
    Serial.print(MyQuad.GetVVel() );
    //Serial.print("\twVel : ");
    Serial.print('\t');
    Serial.print(MyQuad.GetWVel() );
    //Serial.print("\tPAngV: ");
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
    



    /*  double * AccelVec;
  double * AngVelVec;
  double * CompassVec;

  AccelVec = GetAcceleration();
  delay(500);
  CompassVec = GetCompassReadings();
  AngVelVec = GetAngularVelocity();

  Serial.print((double)millis() / 1000.0);
  Serial.print('\t');
  //Serial.print(AccelVec[0]);
  Serial.print(AngVelVec[0]);
  //Serial.print(CompassVec[0]);
  Serial.print('\t');
  //Serial.print(AccelVec[1]);
  //Serial.print(CompassVec[1]);
  Serial.print(AngVelVec[1]);
  Serial.print('\t');
  //Serial.print(AccelVec[2]);
  //Serial.print(CompassVec[2]);
  Serial.print(AngVelVec[2]);
  Serial.print('\n');
  //    delete[] AccelVec;
  delete[] AngVelVec;
  //delete[] CompassVec;*/
}
