/* 
   Quaternion class methods. Let's see if this works. 
   I doubt it will. 
*/

#include <iostream>
#include <math.h>
#include "Quaternion.h"

using namespace std;

Quaternion::Quaternion(double x, double y, double z, double angle)
{
  double magnitude;
  magnitude = sqrt((x * x) + (y * y) + (z * z));
  x /= magnitude;
  y /= magnitude;
  z /= magnitude;
  _w = cos(angle / 2.0);
  _x = x * sin(angle / 2.0);
  _y = y * sin(angle / 2.0);
  _z = z * sin(angle / 2.0);
}

void Quaternion::Set(double x, double y, double z, double angle)
{
  double magnitude;
  magnitude = sqrt((x * x) + (y * y) + (z * z));
  x /= magnitude;
  y /= magnitude;
  z /= magnitude;
  _w = cos(angle / 2.0);
  _x = x * sin(angle / 2.0);
  _y = y * sin(angle / 2.0);
  _z = z * sin(angle / 2.0);
  //cout << "w = " << _w << " x = " << _x << " y = " << _y << " z = " << _z << endl;
}

Quaternion Quaternion::operator*(const Quaternion& qTwo)
{
  double w;
  double x;
  double y;
  double z;
  Quaternion temp(0,0,0,0);

  w = (this->_w * qTwo._w) - (this->_x * qTwo._x) - (this->_y * qTwo._y) - (this->_z * qTwo._z); 
  x = (this->_w * qTwo._x) + (this->_x * qTwo._w) + (this->_y * qTwo._z) - (this->_z * qTwo._y); 
  y = (this->_w * qTwo._y) - (this->_x * qTwo._z) + (this->_y * qTwo._w) + (this->_z * qTwo._x); 
  z = (this->_w * qTwo._z) + (this->_x * qTwo._y) - (this->_y * qTwo._x) + (this->_z * qTwo._w); 

  temp.SetW(w);
  temp.SetX(x);
  temp.SetY(y);
  temp.SetZ(z);

  return temp;
}
