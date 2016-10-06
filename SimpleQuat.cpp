#include <iostream>
#include <math.h>

using namespace std;


//#include "SimpleQuat.h"
void print(double A[4], int length){
    for (int n=0; n<length; n++){
      cout << A[n] << ' ';
    }
    cout << '\n';
}

//don't include angle for magnitude: length-1
double QuatMagCalc(double QuatMatrix[4], int length){
  double QuatMag;
  for (int n=0 ; n<length; n++){
    QuatMag += (QuatMatrix[n]*QuatMatrix[n]);
  }
  return sqrt(QuatMag);
}

/*
//might want to explore using pointers for this
double QuatNormalize(double * QuatMatrix[4], int length, double QuatMag){
  double NormalizedMatrix[length];
    for (int n=0; n<length; n++){
      NormalizedMatrix[n] = (QuatMatrix[n]/QuatMag);
    }
    return []NormalizedMatrix;

        cout << NormalizedMatrix[2];
  return for (int n=0; n<length; n++){
    NormalizedMatrix[n];

} 
*/

int main(){

    double QuatOrientation[4]; //
//    double QuatRotation[4];   // w, z, y, z
    double QuatFinal[4];

//initial Orientation 
    for (int i=0;i<4; i++){
      cout << "Value for QuatOrientation[" << i << "]: ";
      cin >> QuatOrientation[i];
    }


    double quatMag = QuatMagCalc( QuatOrientation,4);
    for (int i=0; i<4; i++){
      QuatOrientation[i] /= quatMag;   //Normalize
    }

    double angle = 2.0* acos(QuatOrientation[4]);
    cout << angle << endl;

 //For figuring out whether mag=1
    double tmag = QuatOrientation[0]+QuatOrientation[1]+QuatOrientation[2]+QuatOrientation[3];
    cout << tmag << "\n" << endl;

//The Rotation. This is where the Euler angles in 3DVector come into play
//We might not be given w, so gotta find angle
//Matrix of local rotation about each axix
                    //{w     x     y     z} //ask for convention?
  double QuatRotation[4] = { (cos(angle/2.0)), (QuatOrientation[1]*sin(angle/2.0)), (QuatOrientation[2]*sin(angle/2.0)), (QuatOrientation[3]*sin(angle/2.0))};

/*
    for (int i=0;i<4; i++){
      cout << "Value for QuatRotation[" << i << "]: ";
      cin >> QuatRotation[i];
    }
   
    double quatMag = QuatMagCalc( QuatRotation,4);
    for (int i=0; i<4; i++){
      QuatRotation[i] /= quatMag;
    }
*/


//Final Quaternion
      QuatFinal[0] = Quat


    print(QuatOrientation,4);
    print(QuatRotation,4);

    


    return 0;
}
