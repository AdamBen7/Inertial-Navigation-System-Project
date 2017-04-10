#include "Kalman.h"

Kalman::Kalman(double dt, double epsilon, double ux, double uy ){

    _StateVecX.resize(6);
	_StateVecZ.resize(6);

	_MatA.resize(6,6);
	_MatB.resize(6,6);
	_MatH.resize(6,6);
	_MatP.resize(6,6);
	_MatQ.resize(6,6);
	_MatR.resize(6,6);
	_KalmanG.resize(6,6); //kalman gain

//Dynamics Matrix
	_MatA << 1.0, 0.0, dt, 0.0, (dt*dt)/2, 0.0,
		    0.0, 1.0, 0.0, dt, 0.0, (dt*dt)/2,
    	    0.0, 0.0, 1.0, 0.0, dt, 0.0,
	        0.0, 0.0, 0.0, 1.0, 0.0, dt,
	        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
	        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

//For now, that's our apparatus
	_MatB.setZero();

	_MatH.setIdentity();

//process noise Matrix
	_MatQ.setZero();
	_MatQ(4,4) = epsilon;
	_MatQ(5,5) = epsilon;

	_MatR.setZero();
	_MatR(0,0) = dt*dt*ux;
	_MatR(1,1) = dt*dt*uy;
	_MatR(2,2) = dt*ux;
	_MatR(3,3) = dt*uy;
	_MatR(4,4) = ux;
	_MatR(5,5) = uy;


//At k=1
	_StateVecX.setZero();
	_StateVecZ.setZero();
	_MatP = _MatR;

//Cleaning the Array of Kalman Gain and MatAnnoying
	_KalmanG.setZero();

	_MatAnnoying.setZero();
}


void Kalman::SetStateVecZ(double ax, double ay){
	_StateVecZ(4) = ax;
	_StateVecZ(5) = ay;
}


Kalman::~Kalman(){};

VectorXd Kalman::KFilter(){
	_StateVecX = _MatA * _StateVecX + _MatB; //MatB*u
	_MatP = _MatA * _MatP * (_MatA.transpose()) + _MatQ;
	_MatAnnoying = (_MatH * _MatP * (_MatH.transpose()) + _MatR);
	_KalmanG = _MatP * _MatH.transpose() * _MatAnnoying.inverse();
	_StateVecX += _KalmanG * (_StateVecZ - _MatH * _StateVecX); //update state vector
	_MatP = (_MatH - _KalmanG * _MatH) * _MatP; //update uncertainty. MatH might be used somewhere else

	return _StateVecX;
}




