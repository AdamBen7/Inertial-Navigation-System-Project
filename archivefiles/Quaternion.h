/*
   Quaternion class, hopefully I understand them enough to do this. Possibly not. 
   So, this may or may not work.
*/

#ifndef Quaternion_h
#define Quaternion_h

#include <math.h>

class Quaternion
{
  public:
    Quaternion(double x, double y, double z, double angle);
    void Set(double x, double y, double z, double angle);
    void SetW(double w) {_w = w;}
    void SetX(double x) {_x = x;}
    void SetY(double y) {_y = y;}
    void SetZ(double z) {_z = z;}
    double GetAngle() {return acos(_w) * 2;}
    double GetXAxis() {return _x / sqrt(1.0 - (_w * _w));}
    double GetYAxis() {return _y / sqrt(1.0 - (_w * _w));}
    double GetZAxis() {return _z / sqrt(1.0 - (_w * _w));}
    Quaternion operator*(const Quaternion&);

  private:
    double _w;
    double _x;
    double _y;
    double _z;

};

#endif
