#include <iostream>
#include "Quadcopter.h"


//probably UpdatePos, UpdateLinearVel, UpdateAngVel

using namespace std;

const double mass = 10;
//might be better to initialize everything here and put as variables like mass
//especially if we want to create QuadcopterObj multiple times with old values

Quadcopter::Quadcopter();
_mass = mass;
_xPos = 0.0;
_yPos = 0.0;
_zPos = 0.0;
_uVel = 0.0;
_vVel = 0.0;
_wVel = 0.0;
_xAccel = 0.0;
_yAccel = 0.0;
_zAccel = 0.0;
_pAngV = 0.0
_qAngV = 0.0
_rAngV = 0.0
_q0 = 1.0;
_q1 = 0.0;
_q2 = 0.0;
_q3 = 0.0;
_time = 0.0; //might not put it in quadcopter obj

}
//consider the destructor later

void const Quadcopter::DisplayState()
{
//for loop with updated position, linear velocity, linear accel vectors
//forloop with updated angular vel/accel vectors
//for loop with quaternion vector
}
