#include <iostream>
#include "math.h"
#include "Quadcopter.h"
//#include "EulertoQuatLib.h"
//Compile with postionupdate.cpp

using namespace std;
void LinearUpdater(Quadcopter &);

int main(){
  Quadcopter MyQuad;

  LinearUpdater(MyQuad);
  MyQuad.DisplayState();

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
