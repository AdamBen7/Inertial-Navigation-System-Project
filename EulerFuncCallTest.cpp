#include <iostream>
#include "math.h"
//#include "vector"
#include "EulertoQuatLib.h"

using namespace std;

/*
void print(double A[3], int length){
    for (int n=0; n<length; n++){
      cout << A[n] << ' ';
    }
}
*/

int main(){
    double EulerMat[3];
    double eulerX, eulerY, eulerZ;
    
    cout << "Value for Heading(EulerX)";
    cin >> eulerX;    
    cout << "Value for Attitude(EulerY)";
    cin >> eulerY;    
    cout << "Value for bank(EulerZ)";
    cin >> eulerZ;    
   
    EulerMat[0] = eulerX;
    EulerMat[1] = eulerY;
    EulerMat[2] = eulerZ;

    double * QuatVec;
    QuatVec = EulertoQuat(EulerMat);

//    std::vector<double> QuatMat;
/*
    double QuatMat[3];
    QuatMat[0] = EulertoQuat(EulerMat,0);
    QuatMat[1] = EulertoQuat(EulerMat,1);
    QuatMat[2] = EulertoQuat(EulerMat,2);
    QuatMat[3] = EulertoQuat(EulerMat,3);
*/

    print(QuatVec,4);


}
