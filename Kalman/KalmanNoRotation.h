#include <cmath>
#include "Eigen30.h"
#include <Eigen/LU>

using Eigen::VectorXd;
using Eigen::MatrixXd;
//using Eigen::Matrix3f;

class Kalman
{
  public:
	Kalman();
	Kalman(double dt, double epsilon, double ux, double uy);
	~Kalman();

	VectorXd GetStateVecX() {return _StateVecX;}
	VectorXd GetStateVecZ() {return _StateVecZ;}

	MatrixXd GetMatA() {return _MatA;}
	MatrixXd GetMatB() {return _MatB;}
	MatrixXd GetMatH() {return _MatH;}
	MatrixXd GetMatI() {return _MatI;}
	MatrixXd GetMatP() {return _MatP;}
	MatrixXd GetMatQ() {return _MatQ;}
	MatrixXd GetMatR() {return _MatR;}
	MatrixXd GetKalmanG() {return _KalmanG;}
	MatrixXd GetMatAnnoying() {return _MatAnnoying;}
	MatrixXd GetMatAdam() { return _MatAdam;}
	void UpdateState(double dt, double ax, double ay);
	VectorXd NoFilter(double dt, double ax, double ay); // #noFilter!<3

	VectorXd KFilter(); //maybe pass in StateVecZ
	MatrixXd Debugger();


  private:
	double _epsilon;
	double _ux;
	double _uy;
	
/*	
	VectorXd _StateVecX(6);
	VectorXd _StateVecZ(6);

	MatrixXd _MatA(6,6);
	MatrixXd _MatB(6,6);
	MatrixXd _MatH(6,6);
	MatrixXd _MatP(6,6);
	MatrixXd _MatQ(6,6);
	MatrixXd _MatR(6,6);
	MatrixXd _KalmanG;(6,6) //kalman gain
	MatrixXd _MatAnnoying(6,6);
*/
	VectorXd _StateVecX;
	VectorXd _StateVecZ;

	MatrixXd _MatA;
	MatrixXd _MatB;
	MatrixXd _MatH;
	MatrixXd _MatI;
	MatrixXd _MatP;
	MatrixXd _MatQ;
	MatrixXd _MatR;
	MatrixXd _KalmanG; //kalman gain
	MatrixXd _MatAnnoying;
	MatrixXd _MatAdam;
};
