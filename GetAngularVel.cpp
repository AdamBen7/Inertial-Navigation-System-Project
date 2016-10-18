//Receive input from gyroscope
//Map it and output angular velocity
//Pass output vector to Rotationupdater
#include <iostream>
using namespace std;

double * GetAngularVelocity(){
  double * AngVelVec = new double [3];
  string input;
  cout << "Insert u-AngVel, v-AngVel, w-AngVel Values: " << endl;
  getline(cin, input);
  cin.sync();
  AngVelVec[0] = stod(input);
  getline(cin, input);
  cin.sync();
  AngVelVec[1] = stod(input);
  getline(cin, input);
  cin.sync();
  AngVelVec[2] = stod(input);

  return AngVelVec;
}
