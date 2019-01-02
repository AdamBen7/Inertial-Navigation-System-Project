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

	_dt = dt;

    _StateVecX.resize(17);
	_StateVecZ.resize(7);//not 15
	_GlobalVelVec.resize(3);
	_GlobalPosVec.resize(3);

	_MatA.resize(17,17);
	_MatB.resize(17,17);
	_MatH.resize(7,17);
	_MatP.resize(17,17);
	_MatQ.resize(17,17);
	_MatR.resize(7,7);

	_MatI.resize(17,17);
	_KalmanG.resize(17,7); //kalman gain

	_MatAnnoying(7,7);

	_MatTransformer.resize(3,3);

	_QuatVec.resize(4);
//Dynamics Matrix

  _MatA <<  1, 0, 0, _dt, 0, 0, _dt*_dt/2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, _dt, 0, 0, _dt*_dt/2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, _dt, 0, 0, _dt*_dt/2, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, _dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 0, 0, 1, 0, 0, _dt, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, _dt, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, _dt, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, _dt, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, _dt, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, _dt, 	
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

			
  _MatB.setZero(); //our apparatus is this for now

  _MatH <<  0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

//measurement noise
  _MatR.setZero();
  _MatR(0,0) = _ux;
  _MatR(1,1) = _uy;
  _MatR(2,2) = _uz;
  _MatR(3,3) = _ugx;
  _MatR(4,4) = _ugx;
  _MatR(5,5) = _ugy;
  _MatR(6,6) = _ugz;

//process noise
  _MatQ.setZero();  
  _MatQ(6,6) = _accelEps;
  _MatQ(7,7) = _accelEps;
  _MatQ(8,8) = _accelEps;
  _MatQ(13,13) = _gyrEps;
  _MatQ(14,14) = _gyrEps;
  _MatQ(15,15) = _gyrEps;
  _MatQ(16,16) = _gyrEps;

  _MatP.setZero();
  _MatP(6,6) = _ux;
  _MatP(7,7) = _uy;
  _MatP(8,8) = _uz;
  _MatP(13,13) = _ugx;
  _MatP(14,14) = _ugx;
  _MatP(15,15) = _ugy;
  _MatP(16,16) = _ugz;

  _MatI.setIdentity();

//At k =1;
  _StateVecX.setZero();
  _StateVecX(9) = 1.0;
  _StateVecZ.setZero();

//Cleaning the Array of Kalman Gain and MatAnnoying
	_KalmanG.setZero();
	_MatAnnoying.setZero();

	_MatTransformer.setIdentity();
}

void Kalman::UpdateState(double dt, double ax, double ay, double az, double gyrX, double gyrY, double gyrZ){

	_q0 = _StateVecX(9);
	_q1 = _StateVecX(10);
    _q2 = _StateVecX(11);
    _q3 = _StateVecX(12);
	_dt = dt;

	gyrX *= M_PI/180.0;
	gyrY *= M_PI/180.0;
	gyrZ *= M_PI/180.0;

	_StateVecZ(0) = ax;
	_StateVecZ(1) = ay;
	_StateVecZ(2) = az;	
	_StateVecZ(3) = gyrX*_q1 + gyrY*_q2 + gyrZ*_q3;  //dq0 --> d_q3
	_StateVecZ(4) = -gyrX*_q0 - gyrZ*_q2 + gyrY*_q3;
	_StateVecZ(5) = -gyrY*_q0 + gyrZ*_q1 - gyrX*_q3;
    _StateVecZ(6) = -gyrZ*_q0 - gyrY*_q1 + gyrX*_q2;

//Incase we want to have varying _dt's.

  _MatA <<  1, 0, 0, _dt, 0, 0, _dt*_dt/2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, _dt, 0, 0, _dt*_dt/2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, _dt, 0, 0, _dt*_dt/2, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, _dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 0, 0, 1, 0, 0, _dt, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, _dt, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, _dt, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, _dt, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, _dt, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, _dt, 	
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
}


Kalman::~Kalman(){};

VectorXd Kalman::KFilter(){

	_StateVecX = _MatA * _StateVecX;// + _MatB; //_MatB*u
	_MatP = _MatA * _MatP * (_MatA.transpose()) + _MatQ;
	_MatAnnoying = (_MatH * _MatP * (_MatH.transpose()) + _MatR);
	_KalmanG = _MatP * _MatH.transpose() * _MatAnnoying.inverse();
	_StateVecX += _KalmanG * (_StateVecZ - _MatH * _StateVecX); //update state vector
	_MatP = (_MatI - _KalmanG * _MatH) * _MatP; //update uncertainty. MatH might be used somewhere else

    _QuatVec << _StateVecX(9), _StateVecX(10), _StateVecX(11), _StateVecX(12);
    _QuatVec /= sqrt(_QuatVec(0)*_QuatVec(0) + _QuatVec(1)*_QuatVec(1) + _QuatVec(2)*_QuatVec(2) + _QuatVec(3)*_QuatVec(3));
	_StateVecX(9) = _QuatVec(0);
	_StateVecX(10) = _QuatVec(1);
	_StateVecX(11) = _QuatVec(2);
	_StateVecX(12) = _QuatVec(3);

	_MatTransformer << _q0*_q0+_q1*_q1-_q2*_q2-_q3*_q3, 2*(_q1*_q2-_q0*_q3), 2*(_q1*_q3+_q0*_q2), 
                      2*(_q1*_q2+_q0*_q3), _q0*_q0-_q1*_q1+_q2*_q2-_q3*_q3, 2*(_q2*_q3-_q0*_q1), 
                      2*(_q1*_q3-_q0*_q2), 2*(_q2*_q3+_q0*_q1), _q0*_q0-_q1*_q1-_q2*_q2+_q3*_q3;

    _GlobalVelVec(0) = _StateVecX(3);
    _GlobalVelVec(1) = _StateVecX(4);
    _GlobalVelVec(2) = _StateVecX(5);

    _GlobalVelVec = _MatTransformer*_GlobalVelVec;
    _GlobalPosVec += _dt * _GlobalVelVec; //scalar-vector multiplication

	_StateVecX(0) = _GlobalPosVec(0);
	_StateVecX(1) = _GlobalPosVec(1);
	_StateVecX(2) = _GlobalPosVec(2);

	return _StateVecX;

}

MatrixXd Kalman::Debugger(){
	return _MatTransformer;
}

/*
//OutofDate. todo later
VectorXd Kalman::NoFilter(double _dt, double ax, double ay, double gyrZ){
	_MatA << 1.0, 0.0, 0.0, _dt, 0.0, 0.0, cos(_psi)*(_dt*_dt)/2, -sin(_psi)*(_dt*_dt)/2,
		    0.0, 1.0, 0.0, 0.0, _dt, 0.0, sin(_psi)*(_dt*_dt)/2,  cos(_psi)*(_dt*_dt)/2,
    	    0.0, 0.0, 1.0, 0.0, 0.0, _dt, 0.0, 0.0,
	        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, cos(_psi)*_dt, -sin(_psi)*_dt,
	        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, sin(_psi)*_dt, cos(_psi)*_dt,
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

