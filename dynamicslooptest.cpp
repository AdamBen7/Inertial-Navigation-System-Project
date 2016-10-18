#include <iostream>
#include "math.h"
#include "Quadcopter.h"
//#include "EulertoQuatLib.h"
//Compile with postionupdate.cpp

using namespace std;
void LinearUpdater(Quadcopter &);

int main(){
  bool moving = true;
  Quadcopter MyQuad;

  while (moving)
  {

    LinearUpdater(MyQuad);
    MyQuad.DisplayState();
    cout << "Keep Moving? 1/0";
    cin >> moving;
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
