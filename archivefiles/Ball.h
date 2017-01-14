/* 
   Header for Ball class, accelerates linearly in 3 dimensions
   Part of "messing around"
*/

#ifndef Ball_h
#define Ball_h

class Ball
{
  public:
    Ball(double xPos, double yPos, double zPos, double mass);
    double GetXPos() {return _xPos;}
    double GetYPos() {return _yPos;}
    double GetZPos() {return _zPos;}
    double GetXVel() {return _xVel;}
    double GetYVel() {return _yVel;}
    double GetZVel() {return _zVel;}
    double GetXAccel() {return _xAccel;}
    double GetYAccel() {return _yAccel;}
    double GetZAccel() {return _zAccel;}
    void SetXAccel(double xAccel) {_xAccel = xAccel;}
    void SetYAccel(double yAccel) {_yAccel = yAccel;}
    void SetZAccel(double zAccel) {_zAccel = zAccel;}
    void BallState(double time);
    void const DisplayStatus(); 
    
  private:
    double _mass;
    double _xPos;
    double _yPos;
    double _zPos;
    double _xVel;
    double _yVel;
    double _zVel;
    double _xAccel;
    double _yAccel;
    double _zAccel;
    double _lastTime;
};
#endif
