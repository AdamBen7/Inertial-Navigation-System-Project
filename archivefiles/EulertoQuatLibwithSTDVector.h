//Given Euler Angles, convert to Quaternion
#include <iostream>
#include <vector>
#include "math.h"
//#include "Quaternion.h" //using Nathan's for now
using namespace std;

void print(double A[4], int length){
    for (int n=0; n<length; n++){
      cout << A[n] << ' ';
    }
}
//testing out std::vector<double>
double QuatMagCalc(std::vector<double> QuatMatrix, int length){
  double QuatMag;
  for (int n=0 ; n<length; n++){
    QuatMag += (QuatMatrix[n]*QuatMatrix[n]);
  }
  return sqrt(QuatMag);
}

//Angles in Radian
std::vector<double> EulertoQuat(double EulerMat[3]){

  double eulerX, eulerY, eulerZ;
  eulerX = EulerMat[1];
  eulerY = EulerMat[2];
  eulerZ = EulerMat[3];

 std::vector<double> FinalQuatMatrix;
  double w1;
  double x1;
  double y1;
  double z1;

  FinalQuatMatrix[0] = 1.0;
  FinalQuatMatrix[1] = 0;
  FinalQuatMatrix[2] = 0;
  FinalQuatMatrix[3] = 0;


  w1 = FinalQuatMatrix[0];
  x1 = FinalQuatMatrix[1];
  y1 = FinalQuatMatrix[2];
  z1 = FinalQuatMatrix[3];

//abbreviation variables
  double cX = cos(eulerX/2);
  double cY = cos(eulerY/2);
  double cZ = cos(eulerZ/2);
  double sX = sin(eulerX/2);
  double sY = sin(eulerY/2);
  double sZ = sin(eulerZ/2);

  double w2 = cX * cY * cZ - sX * sY * sZ;
  double x2 = sX * sY * cZ + cX * cY * sZ;
  double y2 = sX * cY * cZ + cX * sY * sZ;
  double z2 = cX * sY * cZ - sX * cY * sZ;

  double angle = 2.0* acos(w2);  

  FinalQuatMatrix[0] = (w1*w2 - x1*x2 - y1*y2 - z1*z2);
  FinalQuatMatrix[1] = (w1*x2 + x1*w2 + y1*z2 - z1*y2);
  FinalQuatMatrix[2] = (w1*y2 - x1*z2 + y1*w2 + z1*x2);
  FinalQuatMatrix[3] = (w1*z2 + x1*y2 - y1*x2 + z1*w2);

  return FinalQuatMatrix;

}

/*
  double w2 = cX * cY * cZ + sX * sY * sZ;
  double x2 = sX * cY * cZ - cX * sY * sZ;
  double y2 = cX * sY * cZ + sX * cY * sZ;
  double z2 = cX * cY * sZ - sX * sY * cZ;
*/
