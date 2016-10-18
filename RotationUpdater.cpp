#include <iostream>
#include "Quadcopter.h"
#include "math.h"

using namespace std;

void RotationUpdater(Quadcopter & MyQuad)
{
  double dtime = 1.0;

  double q0 = MyQuad.Getq0();
  double q1 = MyQuad.Getq1();
  double q2 = MyQuad.Getq2();
  double q3 = MyQuad.Getq3();

  double p = MyQuad.GetPAngVel();
  double q = MyQuad.GetQAngVel();
  double r = MyQuad.GetRAngVel();

  double dq0 = (p * q1 + q * q2 + r * q3) / (-2.0);
  double dq1 = (-p * q0 + -r * q2 + q * q3) / (-2.0);
  double dq2 = (-q * q0 + r * q1 + -p * q3) / (-2.0);
  double dq3 = (-r * q0 + -q * q1 + p * q2) / (-2.0);

  Setq0(q0 + dq0 * dtime);
  Setq1(q1 + dq1 * dtime);
  Setq2(q2 + dq2 * dtime);
  Setq3(q3 + dq3 * dtime);
  
}
