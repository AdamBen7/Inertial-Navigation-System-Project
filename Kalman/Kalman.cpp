#include "Kalman.h"

Kalman::Kalman(double dt, double epsilon, double ux, double uy ){

	_epsilon = epsilon;
	_ux = ux;
	_uy = uy;

    _StateVecX.resize(6);
	_StateVecZ.resize(6);

	_MatA.resize(6,6);
	_MatB.resize(6,6);
	_MatH.resize(6,6);
	_MatI.resize(6,6);
	_MatP.resize(6,6);
	_MatQ.resize(6,6);
	_MatR.resize(6,6);
	_KalmanG.resize(6,6); //kalman gain
	_MatAdam.resize(6,6);


//Dynamics Matrix
	_MatA << 1.0, 0.0, dt, 0.0, (dt*dt)/2, 0.0,
		    0.0, 1.0, 0.0, dt, 0.0, (dt*dt)/2,
    	    0.0, 0.0, 1.0, 0.0, dt, 0.0,
	        0.0, 0.0, 0.0, 1.0, 0.0, dt,
	        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
	        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

//For now, that's our apparatus
	_MatB.setZero();

//Set to zero with last to validating the acceleration values for state
	_MatAdam.setZero();
	_MatAdam(4,4) = 1.0;
	_MatAdam(5,5) = 1.0;
	/*_MatH.setZero();
	_MatH(4,4) = 1.0;
	_MatH(5,5) = 1.0;*/
	_MatH.setIdentity(); //adam strongly thinks this is wrong and that the above is correct. Nathan disagrees. Adam Lost(did he?).

	_MatI.setIdentity();

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

void Kalman::UpdateState(double dt, double ax, double ay){
	
//Incase we want to have varying dt's.
	_MatA << 1.0, 0.0, dt, 0.0, (dt*dt)/2, 0.0,
		    0.0, 1.0, 0.0, dt, 0.0, (dt*dt)/2,
    	    0.0, 0.0, 1.0, 0.0, dt, 0.0,
	        0.0, 0.0, 0.0, 1.0, 0.0, dt,
	        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
	        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

	_MatR(0,0) = dt*dt*_ux;
	_MatR(1,1) = dt*dt*_uy;
	_MatR(2,2) = dt*_ux;
	_MatR(3,3) = dt*_uy;

    _StateVecZ = _StateVecX;
	_StateVecZ(4) = ax;
	_StateVecZ(5) = ay;

	_StateVecZ *=_MatA;
}


Kalman::~Kalman(){};

VectorXd Kalman::KFilter(){
/*
	_StateVecX = _MatA * _StateVecX + _MatB;
	_StateVecX += -(_StateVecZ - _StateVecX);
*/

	_StateVecX = _MatA * _StateVecX + _MatB; //MatB*u
	_MatP = _MatA * _MatP * (_MatA.transpose()) + _MatQ;
	_MatAnnoying = (_MatH * _MatP * (_MatH.transpose()) + _MatR);
	_KalmanG = _MatP * _MatH.transpose() * _MatAnnoying.inverse();
	_StateVecX += _KalmanG * (_StateVecZ - _MatH * _StateVecX); //update state vector
	_MatP = (_MatI - _KalmanG * _MatH) * _MatP; //update uncertainty. MatH might be used somewhere else

	return _StateVecX;
}

MatrixXd Kalman::Debugger(){
	return _MatH;
}

//not working. todo later
VectorXd Kalman::NoFilter(double dt, double ax, double ay){
	_MatA << 1.0, 0.0, dt, 0.0, (dt*dt)/2, 0.0,
		    0.0, 1.0, 0.0, dt, 0.0, (dt*dt)/2,
    	    0.0, 0.0, 1.0, 0.0, dt, 0.0,
	        0.0, 0.0, 0.0, 1.0, 0.0, dt,
	        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
	        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	_StateVecZ(4) = ax;
	_StateVecZ(5) = ay;

	_StateVecX = _MatA * _StateVecX + _MatB; //MatB*u
	_StateVecX += (_StateVecZ - _MatAdam * _StateVecX); //update state vector
	return _StateVecX;
}


