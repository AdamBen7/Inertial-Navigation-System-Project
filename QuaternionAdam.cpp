// All Class and code in one file for now
#include <math.h>
#include <iostream>
//#include <string>
using namespace std;

class Quatenion{

    Quaternion(double angle, double xComp, double yComp, double zComp);
    Quaternion(double w, double x, double y, double z);

    ~Quaternion();

    void SetW(double inW){_w = inW;}
    void SetX(double inX){_x = inX;}
    void SetY(double inY){_y = inY;}
    void SetZ(double inZ){_z = inZ;}
    void SetWFromAngle(double inAngle){_w = cos(inAngle/2.0);}
    void SetXFromXaxis(double inXaxis, inAngle){_x = inXaxis*sin(inAngle/2.0);}
    void SetYFromYaxis(double inYaxis, inAngle){_y = inYaxis*sin(inAngle/2.0);}
    void SetZFromZaxis(double inZaxis, inAngle){_z = inZaxis*sin(inAngle/2.0);}

    double GetW(){return _w;}
    double GetX(){return _x;}
    double GetY(){return _y;}
    double GetZ(){return _z;}

    double GetRotX(w) = cos(

    void SetQuatMag(double inQuatMag){_quatMag = inQuatMag;}
    double GetQuatMag(){return _quatMag;}

    double CalcQuatMag(w,x,y,z){ _quatMag = sqrt((w*w)+(x*x)+(y*y)+(z*z));}
    
    Quaternion opertator*(const Quaternion&);
        
  private:
    double _w;
    double _x;
    double _y;
    double _z;
    double _quatMag;
    double _quatMatrix[][4] = {_w,_x,_y,_z};
    
};

//Given angle and X,Y,Z components of point
//Decide on which to use:

Quaternion::Quaternion(double angle, double xComp, double yComp, double zComp)
{
    double magnitude;
    magnitude = sqrt((xComp*xComp) + (yComp*yComp) + (zComp*zComp));
    if (magnitude != 1) then
    {
        xComp /= magnitude;
        yComp /= magnitude;
        zComp /= magnitude;
    }
  _w = cos(angle/2.0);
  _x = xComp * sin(angle/2.0);
  _y = yComp * sin(angle/2.0);
  _z = zComp * sin(angle/2.0);
}

/*
Quaternion::Quaternion(): _w (w), _x(x), _y(y), _z(z)
{
}
*/


Quaternion::~Quaternion()
{}

// Maybe Later
/*
Quaternion& Quaternion::operator*(const Quaternion& TempQuat){

}
*/

   

/*
Quaternion::SetQuatMatrix(double w,x,y,z)
{
    _quatMatrix[][4] = { w, x, y, z};
    double magnitude;
    magnitude = sqrt((xComp*xComp) + (yComp*yComp) + (zComp*zComp));
    if (magnitude != 1) then
    {
        xComp /= magnitude;
        yComp /= magnitude;
        zComp /= magnitude;
    }
  _w = cos(angle/2.0);
  _x = xComp * sin(angle/2.0);
  _y = yComp * sin(angle/2.0);
*/

int main(){
  double angle, x, y, z;
  Quaternion Orientation(1,0,0,0);
  Quaternion Rotation(1,0,0,0);
    
  while(true){
    cout << orientation.GetAngle() <<
    

}
