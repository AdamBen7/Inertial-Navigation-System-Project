/*
   Runs a loop to test the Ball object
   WARNING: HIGHLY UNSTABLE CODE USED FOR TESTING ONLY
*/

#include <iostream>
#include <cstdlib>
#include <string>
#include "Ball.h"

using namespace std;

int main()
{
  string xAccel;
  string yAccel;
  string zAccel;
  string time;
  string yesOrNo;
  bool done;
  done = false;

  Ball testBall = Ball(0.0,0.0,0.0,1.5);

  while (!done)
  {
    cout << "Acceleration in x direction: ";
    getline(cin, xAccel);
    cin.sync(); 
    cout << "Acceleration in y direction: ";
    getline(cin, yAccel);
    cin.sync(); 
    cout << "Acceleration in z direction: ";
    getline(cin, zAccel);
    cin.sync(); 
    cout << "Current time from start (in seconds): ";
    getline(cin, time);
    cin.sync(); 
  
    testBall.SetXAccel(stod(xAccel));
    testBall.SetYAccel(stod(yAccel));
    testBall.SetZAccel(stod(zAccel));
    testBall.BallState(stod(time));
    testBall.DisplayStatus();

    cout << "Are you done (y/n)? ";
    getline(cin, yesOrNo);
    cin.sync();
    if (yesOrNo[0] == 'y')
      done = true;
  }

  return 0;
}
