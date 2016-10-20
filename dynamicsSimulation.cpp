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
  double time = 0.0;

  while (time < 1.0)
  {
    time += .001;
    MyQuad.SetTime(time);
    RotationUpdater(MyQuad);
    LinearUpdater(MyQuad);
    MyQuad.DisplayState();
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
