//Fetches accelerometer's raw data
//Maps to acceleration value
//Gets calls directly by UpdatePos
//Decided not to store accel values in Quadcopter.h due to efficiency of code.

#include <iostream>
#include <string>

using namespace std;

double* GetAcceleration(){
  double * AccelVec = new double [3];
  /*
  string input;
  cout << "Insert X-Accel, Y-Accel, Z-Accel Values :" << endl;
  getline(cin, input);
  cin.sync();
  AccelVec[0] = stod(input);
  getline(cin, input);
  cin.sync();
  AccelVec[1] = stod(input);
  getline(cin, input);
  cin.sync();
  AccelVec[2] = stod(input);
  */

  AccelVec[0] = 1.0;
  AccelVec[1] = 0.0;
  AccelVec[2] = 0.0;

  return AccelVec;
}
