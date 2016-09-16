/*
   Code to see if the quternion class works.
   Then again, I don't fully understand quaternions, so will I be able to tell?

   What, quaternions?
   One real part and three complex?
   I guess that makes sense?
*/

#include <iostream>
#include <string>
#include <math.h>
#include "Quaternion.h"

using namespace std;

int main()
{
  string xAxis;
  string yAxis;
  string zAxis;
  string angle;
  Quaternion orientation(1,0,0,0);
  Quaternion rotation(1,0,0,0);

  while(true)
  {
    cout << "Current orientation: ";
    cout << orientation.GetAngle() << " radians around <";
    cout << orientation.GetXAxis() << ", " << orientation.GetYAxis() << ", " << orientation.GetZAxis() << ">";
    cout << endl;

    cout << "\nAre you done?";
    getline(cin, xAxis);
    cin.sync();
    if(xAxis[0] == 'y')
      break;

    cout << "\nEnter new rotation axis:\nX: ";
    getline(cin, xAxis);
    cin.sync();
    cout << "Y: ";
    getline(cin, yAxis);
    cin.sync();
    cout << "Z: ";
    getline(cin, zAxis);
    cin.sync();
    cout << "\nEnter rotation angle in radians: ";
    getline(cin, angle);
    cin.sync();

    rotation.Set(stod(xAxis), stod(yAxis), stod(zAxis), stod(angle));
    orientation = rotation * orientation;    
  }
}
    
