#include <iostream>
#include "math.h"
#include "Quadcopter.h"
#include "EulertoQuatLib.h"

using namespace std;
//Position Update Matrix 

int main(){
  //  Quadcopter Quad;

  double mass = 10.0;
  double grav = 9.81;
  double weight = mass * grav;

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

  //bodyframe linear vel
  double u = 0;
  double v = 0;
  double w = 0;

  double LVelVec[3] = {u, v, w};

  //Angular Velocity, Body Frame
  double p = 0;
  double q = 0;
  double r = 0;

  //LinearForces Experienced
  double X = 0;
  double Y = 0;
  double Z = 0;

  double BMat[][3] =
  {{q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2-q0*q3),         2*(q1*q3+q0*q2)},
  {2.0*(q1*q2+q0*q3),        q0*q0-q1*q1+q2*q2-q3*q3, 2.0*(q2*q3-q0*q1)}, 
  {2.0*(q1*q3-q0*q2),        2.0*(q2*q3+q0*q1),       q0*q0-q1*q1-q2*q2+q3*q3}};

  double dxPos = BMat[1][0] * LVelVec[1] + BMat[1][1] * LVelVec[2] + BMat[1][2] * LVelVec[3] ;
  double dyPos = BMat[2][0] * LVelVec[1] + BMat[2][1] * LVelVec[2] + BMat[2][2] * LVelVec[3] ;
  double dzPos = BMat[3][0] * LVelVec[1] + BMat[3][1] * LVelVec[2] + BMat[3][2] * LVelVec[3] ;

  //derivative of x,y,z
  double dPosVector[3] = {dxPos, dyPos, dzPos};

  //equations of motion

  double du = r*v -q*w + (X/mass) - grav*sin(eulerX);
  double dv = -r*u+p*q + (Y/mass) - grav*cos(eulerX)*sin(eulerY);
  double dw = q*u-p*v  + (Z/mass) - grav*cos(eulerX)*cos(eulerY);

  //acceleration in body frame
  double dVelVector[3] = {du, dv, dw};

  print(dPosVector,3);

  delete[] QuatVec;
  return 0;
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
