/* 
   Header for Ball class, accelerates linearly in 3 dimensions
   Part of "messing around"
*/

#ifndef Ball_h
#define Ball_h

class Ball
{
  public:
    Ball(double xPos, double yPos, double zPos);
    void accel(double xAccel, double yAccel, double zAccel, double time);
    void displayStatus(); 
    
  private:
    double _xPos;
    double _yPos;
    double _zPos;
    double _xVel;
    double _yVel;
    double _zVel;
    double _lastTime;
};
#endif
