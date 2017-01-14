#include <iostream>
#include "math.h"
#include "Quadcopter.h"
//#include "EulertoQuatLib.h"
//Compile with postionupdate.cpp

using namespace std;
void LinearUpdater(Quadcopter &);
void RotationUpdater(Quadcopter &);

int main(){
  bool moving = true;
  Quadcopter MyQuad;
  string input;

  while (moving)
  {
    cout << "Enter time from start in seconds: ";
    getline(cin, input);
    cin.sync();
    MyQuad.SetTime(stod(input));
    RotationUpdater(MyQuad);
    LinearUpdater(MyQuad);
    MyQuad.DisplayState();
    cout << "Keep Moving? 1/0";
    getline(cin, input);
    cin.sync();
    moving = stod(input);
    cout << endl;

  }
    return 0;
}
//    return  posMat;
/*
  print(BTransMat[1],3);
  cout << endl;
  print(BTransMat[2],3);
  cout << endl;
  print(BTransMat[3],3);
  cout << endl;
*/
