/*
  This function takes a quaternion, the three angular velocity terms, p q r, and a time interval
  and returns a new quaternion.
*/

#include <iostream>
#include "quaternion.h"

Quaternion quatUpdate(Quaternion oldQuat, double p, double q, double r, double timeElapsed)
{
  double oldW = oldQuat.GetW();
  double oldX = oldQuat.GetX();
  double oldY = oldQuat.GetY();
  double oldZ = oldQuat.GetZ();

  double w;
  double x;
  double y;
  double z;

  w = oldW + (timeElapsed * ((p * oldX) + (q * oldY) + (r * oldZ)) / (-2));
  x = oldX + (timeElapsed * ((-p * oldW) + (-r * oldY) + (q * oldZ)) / (-2));
  y = oldY + (timeElapsed * ((-q * oldW) + (r * oldX) + (-p * oldZ)) / (-2));
  z = oldZ + (timeElapsed * ((-r * oldW) + (-q * oldX) + (p * oldY)) / (-2));

  return Quaternion(w, x, y, z);
}
