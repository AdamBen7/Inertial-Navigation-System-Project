#include <iostream>
#include "math.h"
#include "Quadcopter.h"
#include "EulertoQuatLib.h"

using namespace std;
//Position Update Matrix 
double * GetAcceleration();

void LinearUpdater(Quadcopter & MyQuad){

  //  Quadcopter Quad;

  double mass = 10.0;
  double grav = 9.81;
  double weight = mass * grav;
  double dtime = MyQuad.GetDTime();
//I'm past patiently waitin', I'm passionately smashin' every expectation, every action's an act of creation
  double * AccelVec;
  AccelVec = GetAcceleration();

  double xAccel = AccelVec[0]; //xAccel
  double yAccel = AccelVec[1]; //yAccel
  double zAccel = AccelVec[2]; //zAccel

/*
  double EulerVec[3];
  double eulerX = 0; 
  double eulerY = 0;
  double eulerZ = 0;

  double * QuatVec;
  QuatVec = EulertoQuat(EulerVec);

  //Quat Orientation
  double q0 = QuatVec[0];
  double q1 = QuatVec[1];
  double q2 = QuatVec[2];
  double q3 = QuatVec[3];
*/

  double q0 = MyQuad.Getq0();
  double q1 = MyQuad.Getq1();
  double q2 = MyQuad.Getq2();
  double q3 = MyQuad.Getq3();

  double xPos = MyQuad.GetXPos();
  double yPos = MyQuad.GetYPos();
  double zPos = MyQuad.GetZPos();

  //bodyframe linear vel
  double u = MyQuad.GetUVel();
  double v = MyQuad.GetVVel();
  double w = MyQuad.GetWVel();

  double LVelVec[3] = {u, v, w};

  //Angular Velocity, Body Frame
  double p = MyQuad.GetPAngVel();
  double q = MyQuad.GetQAngVel();
  double r = MyQuad.GetRAngVel();

  //LinearForces Experienced
  double X = 0.0;
  double Y = 0.0;
  double Z = 0.0;

  double BMat[][3] =
  {{q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2-q0*q3),         2*(q1*q3+q0*q2)},
  {2.0*(q1*q2+q0*q3),        q0*q0-q1*q1+q2*q2-q3*q3, 2.0*(q2*q3-q0*q1)}, 
  {2.0*(q1*q3-q0*q2),        2.0*(q2*q3+q0*q1),       q0*q0-q1*q1-q2*q2+q3*q3}};

  double dxPos = BMat[0][0] * LVelVec[0] + BMat[0][1] * LVelVec[1] + BMat[0][2] * LVelVec[2] ;
  double dyPos = BMat[1][0] * LVelVec[0] + BMat[1][1] * LVelVec[1] + BMat[1][2] * LVelVec[2] ;
  double dzPos = BMat[2][0] * LVelVec[0] + BMat[2][1] * LVelVec[1] + BMat[2][2] * LVelVec[2] ;

  //derivative of x,y,z
  double dPosVector[3] = {dxPos, dyPos, dzPos};

  //equations of motion


  double du = r*v - q*w + AccelVec[0];
  double dv = -r*u + p*w + AccelVec[1];
  double dw = q*u - p*v  + AccelVec[2];

/* //Commented out atm since we might not need to touch euler angles. 
  double du = r*v -q*w + (X/mass) - grav*sin(eulerX);
  double dv = -r*u+p*q + (Y/mass) - grav*cos(eulerX)*sin(eulerY);
  double dw = q*u-p*v  + (Z/mass) - grav*cos(eulerX)*cos(eulerY);

  //acceleration in body frame
  double dVelVector[3] = {du, dv, dw};
*/
  //cerr << "Hi everyone!"; 
  //cout << dPosVector[0] << endl;
  // Update Quadcopter Object
  MyQuad.SetXPos(xPos + dPosVector[0]*dtime);
  MyQuad.SetYPos(yPos + dPosVector[1]*dtime);
  MyQuad.SetZPos(zPos + dPosVector[2]*dtime);

  MyQuad.SetUVel(u + du*dtime);
  MyQuad.SetVVel(v + dv*dtime);
  MyQuad.SetWVel(w + dw*dtime);

  delete[] AccelVec;
//  print(dPosVector,3);

//  delete[] QuatVec;
}
//    return  posMat;
/*
  print(BTransMat[1],3);
  cout << endl;
  print(BTransMat[2],3);
  cout << endl;
  print(BTransMat[3],3);
  cout << endl;
*/
