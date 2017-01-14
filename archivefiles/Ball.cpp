/*
   Methods for Ball class, accelerates linearly in 3 dimensions
   For "messing around"
*/

#include <iostream>
#include "Ball.h"

using namespace std;

const double mass = .1;

Ball::Ball(double xPos, double yPos, double zPos, double mass)
{
  _mass = mass;
  _xPos = xPos;
  _yPos = yPos;
  _zPos = zPos;
  _xVel = 0.0;
  _yVel = 0.0;
  _zVel = 0.0;
  _xAccel = 0.0;
  _lastTime = 0.0;
}

void Ball::BallState(double time)
{
  double elapsedTime = time - _lastTime;
  _xPos += (_xVel * elapsedTime) + (_xAccel * elapsedTime * elapsedTime / 2.0); 
  _yPos += (_yVel * elapsedTime) + (_yAccel * elapsedTime * elapsedTime / 2.0); 
  _zPos += (_zVel * elapsedTime) + (_zAccel * elapsedTime * elapsedTime / 2.0); 
  _xVel += (_xAccel * elapsedTime);
  _yVel += (_yAccel * elapsedTime);
  _zVel += (_zAccel * elapsedTime);
  _lastTime = time;
}

void const Ball::DisplayStatus()
{
  cout << "Position (x,y,z): (" << _xPos << "," << _yPos << "," << _zPos << ")" << endl;  
  cout << "Velocity <x,y,z>: <" << _xVel << "," << _yVel << "," << _zVel << ">" << endl;
}
