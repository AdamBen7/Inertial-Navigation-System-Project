//Given Euler Angles, convert to Quaternion
#include <iostream>
#include "math.h"
//#include "Quaternion.h" //using Nathan's for now
using namespace std;

void print(double A[4], int length){
    for (int n=0; n<length; n++){
      cout << A[n] << ' ';
    }
}

double QuatMagCalc(double QuatMatrix[4], int length){
  double QuatMag;
  for (int n=0 ; n<length; n++){
    QuatMag += (QuatMatrix[n]*QuatMatrix[n]);
  }
  return sqrt(QuatMag);
}

//Angles in Radian
int main(){
    
int count = 0;
bool Rotating = true;
double eulerX, eulerY, eulerZ;
double FinalQuatMatrix[4];
double w1;
double x1;
double y1;
double z1;

FinalQuatMatrix[0] = 1.0;
FinalQuatMatrix[1] = 0;
FinalQuatMatrix[2] = 0;
FinalQuatMatrix[3] = 0;

while (Rotating == true){

  w1 = FinalQuatMatrix[0];
  x1 = FinalQuatMatrix[1];
  y1 = FinalQuatMatrix[2];
  z1 = FinalQuatMatrix[3];

    cout << "Value for Heading(EulerX)";
    cin >> eulerX;    
    cout << "Value for Attitude(EulerY)";
    cin >> eulerY;    
    cout << "Value for bank(EulerZ)";
    cin >> eulerZ;    

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

  print(FinalQuatMatrix,4); 
  cout << "\n Continue? Enter 1/0: ";
  cin >> Rotating;
  cout << endl;
  }
  
/*  
    for (int i=0;i<4; i++){
      cout << "Value for QuatOrientation[" << i << "]: ";
      cin >> QuatMat[i];
    }
*/
  
  return 0;

}

/*
  double w2 = cX * cY * cZ + sX * sY * sZ;
  double x2 = sX * cY * cZ - cX * sY * sZ;
  double y2 = cX * sY * cZ + sX * cY * sZ;
  double z2 = cX * cY * sZ - sX * sY * cZ;
*/
