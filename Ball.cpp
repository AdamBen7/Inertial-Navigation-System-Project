/*
   Methods for Ball class, accelerates linearly in 3 dimensions
   For "messing around"
*/

#include <iostream>
#include "Ball.h"

using namespace std;

Ball::Ball(double xPos, double yPos, double zPos)
{
  _xPos = xPos;
  _yPos = yPos;
  _zPos = zPos;
  _xVel = 0.0;
  _yVel = 0.0;
  _zVel = 0.0;
  _lastTime = 0.0;
}

void Ball::accel(double xAccel, double yAccel, double zAccel, double time)
{
  double elapsedTime = time - _lastTime;
  _xPos += (_xVel * elapsedTime) + (xAccel * elapsedTime * elapsedTime / 2.0); 
  _yPos += (_yVel * elapsedTime) + (yAccel * elapsedTime * elapsedTime / 2.0); 
  _zPos += (_zVel * elapsedTime) + (zAccel * elapsedTime * elapsedTime / 2.0); 
  _xVel += (xAccel * elapsedTime);
  _yVel += (yAccel * elapsedTime);
  _zVel += (zAccel * elapsedTime);
  _lastTime = time;
}

void Ball::displayStatus()
{
  cout << "Position (x,y,z): (" << _xPos << "," << _yPos << "," << _zPos << ")" << endl;  
  cout << "Velocity <x,y,z>: <" << _xVel << "," << _yVel << "," << _zVel << ">" << endl;
}
