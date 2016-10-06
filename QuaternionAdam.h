#include <math.h>


class Quatenion{

    Quaternion();
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
