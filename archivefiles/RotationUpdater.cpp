#include <iostream>
#include "Quadcopter.h"
#include "math.h"

using namespace std;
double * GetAngularVelocity();

void RotationUpdater(Quadcopter & MyQuad)
{
  double dtime = MyQuad.GetDTime();
  
  double * AngVelVec;
  AngVelVec = GetAngularVelocity();

  double q0 = MyQuad.Getq0();
  double q1 = MyQuad.Getq1();
  double q2 = MyQuad.Getq2();
  double q3 = MyQuad.Getq3();
 
  //cerr << "q2: " << q2;
  //cerr << "q3: " << q3;

  double p = MyQuad.GetPAngVel();
  double q = MyQuad.GetQAngVel();
  double r = MyQuad.GetRAngVel();

  //cerr << "p :" << p << endl;
  //cerr << "q :" << q << endl;
  //cerr << "r :" << r << endl;

  double dq0 = (p * q1 + q * q2 + r * q3) / (-2.0);
  double dq1 = ((-1 * p * q0) + (-1 * r * q2) + (q * q3)) / (-2.0);
  double dq2 = ((-1 * q * q0) + (r * q1) + (-1 * p * q3)) / (-2.0);
  double dq3 = ((-1 * r * q0) + (-1 * q * q1) + (p * q2)) / (-2.0);

  //cerr << "dq2: " << dq2 << endl;
  //cerr << "dq3: " << dq3 << endl;

  MyQuad.Setq0(q0 + dq0 * dtime);
  MyQuad.Setq1(q1 + dq1 * dtime);
  MyQuad.Setq2(q2 + dq2 * dtime);
  MyQuad.Setq3(q3 + dq3 * dtime);

  MyQuad.SetPAngVel(AngVelVec[0]);
  MyQuad.SetQAngVel(AngVelVec[1]);
  MyQuad.SetRAngVel(AngVelVec[2]);
 
  delete[] AngVelVec;
}
