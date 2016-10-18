#include <iostream>
#include "Quadcopter.h"


//probably UpdatePos, UpdateLinearVel, UpdateAngVel

using namespace std;

const double mass = 10;
//might be better to initialize everything here and put as variables like mass
//especially if we want to create QuadcopterObj multiple times with old values

Quadcopter::Quadcopter(){
_mass = mass;
_xPos = 0.0;
_yPos = 0.0;
_zPos = 0.0;
_uVel = 0.0;
_vVel = 0.0;
_wVel = 0.0;
/*
_xAccel = 0.0;
_yAccel = 0.0;
_zAccel = 0.0;
*/
_pAngV = 0.0;
_qAngV = 0.0;
_rAngV = 0.0;
_q0 = 1.0;
_q1 = 0.0;
_q2 = 0.0;
_q3 = 0.0;
_time = 0.0; //might not put it in quadcopter obj

}
//consider the destructor later

Quadcopter::~Quadcopter(){}

void const Quadcopter::DisplayState()
{
  cout << "xPos : " << _xPos << endl;
  cout << "yPos : " << _yPos << endl;
  cout << "zPos : " << _zPos << endl;
  cout << "uVel : " << _uVel << endl;
  cout << "vVel : " << _vVel << endl;
  cout << "wVel : " << _wVel << endl;
  cout << "PAngV: " << _pAngV << endl;
  cout << "QAngV: " << _qAngV << endl;
  cout << "RAngV: " << _rAngV << endl;
  cout << "Quat0: " << _q0 << endl;
  cout << "Quat1: " << _q1 << endl;
  cout << "Quat2: " << _q2 << endl;
  cout << "Quat3: " << _q3 << endl;
}

//for loop with updated position, linear velocity, linear accel vectors
//forloop with updated angular vel/accel vectors
//for loop with quaternion vector

