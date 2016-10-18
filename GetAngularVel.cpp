//Receive input from gyroscope
//Map it and output angular velocity
//Pass output vector to Rotationupdater
#include <iostream>
using namespace std;

double * GetAngularVelocity(){
  double * AngVelVec = new double [3];
  cout << "Insert u-AngVel, v-AngVel, w-AngVel Values: " << endl;
  cin >> AngVelVec[0];
  cin >> AngVelVec[1];
  cin >> AngVelVec[2];
  
  return AngVelVec;
}
