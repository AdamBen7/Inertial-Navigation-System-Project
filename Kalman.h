#include <cmath>
//#include "stlport.h"
#include "Eigen30.h"
#include <Eigen/LU>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3f;
//using Eigen::Matrix

VectorXd StateVecX(6);
VectorXd StateVecZ(6);

MatrixXd MatA(6,6);
MatrixXd MatB(6,6);
MatrixXd MatH(6,6);
MatrixXd MatP(6,6);
MatrixXd MatQ(6,6);
MatrixXd MatR(6,6);
MatrixXd KalmanG(6,6); //kalman gain
MatrixXd MatAnnoying(6,6);

double dt = 5.0;
double epsilon;
double ux,uy;

void setup() {
  Serial.begin(9600);
  delay(500);
  /*
xk(0,0) = 0.0;
xk(1,0) = 0.0; 
xk(0,1) = 0.0;
xk(1,1) = 0.0;*/
/*xk << 0.0, 1.0,
      2.0, 3.0;*/

//StateVec << 3.0, 1.0, 1.0, 1.0, 1.0, 5.0;

//Dynamics Matrix
MatA << 1.0, 0.0, dt, 0.0, (dt*dt)/2, 0.0,
        0.0, 1.0, 0.0, dt, 0.0, (dt*dt)/2,
        0.0, 0.0, 1.0, 0.0, dt, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, dt,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

//For now, that's our apparatus
MatB.setZero();

MatH.setIdentity();

//process noise Matrix
epsilon = 2.0;
MatQ.setZero();
MatQ(4,4) = epsilon;
MatQ(5,5) = epsilon;

//Measurement Noise Matrix
ux = 4.0;
uy = 3.0;

MatR.setZero();
MatR(0,0) = dt*dt*ux;
MatR(1,1) = dt*dt*uy;
MatR(2,2) = dt*ux;
MatR(3,3) = dt*uy;
MatR(4,4) = ux;
MatR(5,5) = uy;


//At k=1
StateVecX.setZero();
StateVecZ.setZero();
MatP = MatR;

//Cleaning the Array of Kalman Gain
KalmanG.setZero();

MatAnnoying.setZero();
}

void loop() {

StateVecX = MatA * StateVecX + MatB; //MatB*u
MatP = MatA * MatP * (MatA.transpose()) + MatQ;
MatAnnoying = ((MatH.transpose()) * (MatH * MatP * (MatH.transpose()) + MatR));
KalmanG = MatP * MatAnnoying.inverse();
StateVecX += KalmanG * (StateVecZ - MatH * StateVecX); //update state vector
MatP = (MatH - KalmanG * MatH) * MatP; //update uncertainty. MatH might be used somewhere else

//  MatA(0,2) = dt;
  Serial.println("Looped");
//  void matprint(const byte *m, ...);
//  Serial.print(xk(1,1));
  printArray(StateVecX);
//  printArray(MatA);
//  printArray(MatH);
//  printArray(MatQ);
//  printArray(MatR);
//  while(true){};
}


//Polymorphic Printing! Woohoo!
void printArray(Eigen::MatrixXd Mat){
  for(size_t i=0; i<Mat.rows(); i++){
    for (size_t j=0; j<Mat.cols(); j++){
      Serial.print(Mat(i,j));
      Serial.print('\t');
      }
    Serial.print('\n');
    } 
Serial.print('\n');
}

void printArray(Eigen::VectorXd Vec){

  for (size_t j=0; j<(Vec.size()); j++){
    Serial.print(Vec(j));
    Serial.print('\t');
    Serial.print('\n');
    } 
Serial.print('\n');
}

