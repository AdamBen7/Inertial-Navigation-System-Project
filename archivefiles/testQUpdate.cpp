/*
  This program is to test the updateQuaternion function.
*/

#include <iostream>
#include "quaternion.h"
  
using namespace std;

Quaternion quatUpdate(Quaternion quat, double p, double q, double r, double elapsedTime);

int main()
{
  string input;
  double p;
  double q;
  double r;
  double elapsedTime;
  Quaternion myQuat(1,0,0,0);

  cout << "Enter p: ";
  getline(cin, input);
  cin.sync();
  p = stod(input);

  cout << "Enter q: ";
  getline(cin, input);
  cin.sync();
  q = stod(input);
  
  cout << "Enter r: ";
  getline(cin, input);
  cin.sync();
  r = stod(input);

  cout << "Enter time elapsed: ";
  getline(cin, input);
  cin.sync();
  elapsedTime = stod(input);

  myQuat = quatUpdate(myQuat, p, q, r, elapsedTime);

  cout << "Current myQuat: ";
  cout << myQuat.GetAngle() << " radians around <";
  cout << myQuat.GetXAxis() << ", " << myQuat.GetYAxis() << ", " << myQuat.GetZAxis() << ">";
  cout << endl;

}
