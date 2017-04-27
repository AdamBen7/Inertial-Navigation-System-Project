#include "Kalman.h"
#include "Trigd.h"

Kalman::Kalman(double dt, double accelEps, double gyrEps, double ux, double uy, double uz, double ugx, double ugy, double ugz){
	_accelEps = accelEps;
	_gyrEps = gyrEps;
	_ux = ux;
	_uy = uy;
	_uz = uz;
	_ugx = ugx;
	_ugy = ugy;
	_ugz = ugz;//do not set to 0

	_Psi = 0;
	_Theta = 0;
	_Phi = 0;
/*
	_ux = 0.25;
	_uy = 0.25;
	_uz = 0.25;
	_ugx = 0.25;
	_ugy = 0.25;
	_ugz = 0.25;//do not set to 0
*/


    _StateVecX.resize(15);
	_StateVecZ.resize(6);//not 15

	_MatA.resize(15,15);
	_MatB.resize(15,15);
	_MatH.resize(6,15);
	_MatP.resize(15,15);
	_MatQ.resize(15,15);
	_MatR.resize(6,6);

	_MatI.resize(15,15);
	_KalmanG.resize(15,6); //kalman gain

	_MatAnnoying(6,6);

//Dynamics Matrix

  _MatA <<  1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 	
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
			
  _MatB.setZero(); //our apparatus is this for now

  _MatH <<  0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

//measurement noise
  _MatR.setZero();
  _MatR(0,0) = _ux;
  _MatR(1,1) = _uy;
  _MatR(2,2) = _uz;
  _MatR(3,3) = _ugx;
  _MatR(4,4) = _ugy;
  _MatR(5,5) = _ugz;

//process noise
  _MatQ.setZero();  
  _MatQ(6,6) = _accelEps;
  _MatQ(7,7) = _accelEps;
  _MatQ(8,8) = _accelEps;
  _MatQ(12,12) = _gyrEps;
  _MatQ(13,13) = _gyrEps;
  _MatQ(14,14) = _gyrEps;

  _MatP.setZero();
  _MatP(6,6) = _ux;
  _MatP(7,7) = _uy;
  _MatP(8,8) = _uz;
  _MatP(12,12) = _ugx;
  _MatP(13,13) = _ugy;
  _MatP(14,14) = _ugz;

  _MatI.setIdentity();

//At k =1;
  _StateVecX.setZero();
  _StateVecZ.setZero();

//Cleaning the Array of Kalman Gain and MatAnnoying
	_KalmanG.setZero();
	_MatAnnoying.setZero();

}

void Kalman::UpdateState(double dt, double ax, double ay, double az, double gyrX, double gyrY, double gyrZ){

    _Phi = _StateVecX(9);
	_Theta = _StateVecX(10);
	_Psi = _StateVecX(11);

	_StateVecZ(0) = ax;
	_StateVecZ(1) = ay;
	_StateVecZ(2) = az;
	_StateVecZ(3) = gyrX + gyrY*sind(_Phi)*tand(_Theta) + gyrZ*cosd(_Phi)*tand(_Theta); //phidot
	_StateVecZ(4) = gyrY*cosd(_Phi) - gyrZ*sind(_Phi);
	_StateVecZ(5) = gyrY*sind(_Phi)*(1/cosd(_Theta)) + gyrZ*cosd(_Phi)*(1/cosd(_Theta));//make sure Secant is sec()

//Incase we want to have varying dt's.

  _MatA <<  1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 	
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

//	_StateVecZ *=_MatA;
}


Kalman::~Kalman(){};

VectorXd Kalman::KFilter(){
/*
	_StateVecX = _MatA * _StateVecX + _MatB;
	_StateVecX += -(_StateVecZ - _StateVecX);
*/

	_StateVecX = _MatA * _StateVecX;// + _MatB; //_MatB*u
	_MatP = _MatA * _MatP * (_MatA.transpose()) + _MatQ;
	_MatAnnoying = (_MatH * _MatP * (_MatH.transpose()) + _MatR);
	_KalmanG = _MatP * _MatH.transpose() * _MatAnnoying.inverse();
	_StateVecX += _KalmanG * (_StateVecZ - _MatH * _StateVecX); //update state vector
	_MatP = (_MatI - _KalmanG * _MatH) * _MatP; //update uncertainty. MatH might be used somewhere else

	return _StateVecX;
}

MatrixXd Kalman::Debugger(){
	return _MatA;
}

/*
//OutofDate. todo later
VectorXd Kalman::NoFilter(double dt, double ax, double ay, double gyrZ){
	_MatA << 1.0, 0.0, 0.0, dt, 0.0, 0.0, cos(_psi)*(dt*dt)/2, -sin(_psi)*(dt*dt)/2,
		    0.0, 1.0, 0.0, 0.0, dt, 0.0, sin(_psi)*(dt*dt)/2,  cos(_psi)*(dt*dt)/2,
    	    0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
	        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, cos(_psi)*dt, -sin(_psi)*dt,
	        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, sin(_psi)*dt, cos(_psi)*dt,
	        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
	        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
	        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

	_StateVecZ(5) = gyrZ;
	_StateVecZ(6) = ax;
	_StateVecZ(7) = ay;
	// SetPsi(_StateVecX(2)); outdated

	_StateVecX = _MatA * _StateVecX + _MatB; //MatB*u
	_StateVecX += (_StateVecZ - _MatAdam * _StateVecX); //update state vector
	return _StateVecX;
}*/

