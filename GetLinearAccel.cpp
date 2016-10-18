//Fetches accelerometer's raw data
//Maps to acceleration value
//Gets calls directly by UpdatePos
//Decided not to store accel values in Quadcopter.h due to efficiency of code.

#include <iostream>
using namespace std;

double* GetAcceleration(){
  double * AccelVec = new double [3];
  cout << "Insert X-Accel, Y-Accel, Z-Accel Values :" << endl;
  cin >> AccelVec[0];
  cin >> AccelVec[1];
  cin >> AccelVec[2];

  return AccelVec;
}
