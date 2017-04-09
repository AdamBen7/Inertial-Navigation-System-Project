#include <math.h>
//#include <fstream>
//class Motor;

using namespace std;

class Quadcopter
{

  public:
    Quadcopter();
    ~Quadcopter();

    double GetXPos() {return _xPos;}
    double GetYPos() {return _yPos;}
    double GetZPos() {return _zPos;}

    void SetXPos(double xPos) {_xPos = xPos;}
    void SetYPos(double yPos) {_yPos = yPos;}
    void SetZPos(double zPos) {_zPos = zPos;}
    
    double GetdXPos() {return _dxPos;}
    double GetdYPos() {return _dyPos;}
    double GetdZPos() {return _dzPos;}

    void SetdXPos(double dxPos) {_dxPos = dxPos;}
    void SetdYPos(double dyPos) {_dyPos = dyPos;}
    void SetdZPos(double dzPos) {_dzPos = dzPos;}

    double GetPAngVel() {return _pAngV;}
    double GetQAngVel() {return _qAngV;}
    double GetRAngVel() {return _rAngV;}
   
    void SetPAngVel(double pAngV) { _pAngV = pAngV;}
    void SetQAngVel(double qAngV) { _qAngV = qAngV;}
    void SetRAngVel(double rAngV) { _rAngV = rAngV;}

    double GetGravityX(){ return _gaX;}
    double GetGravityY(){ return _gaY;}
    double GetGravityZ(){ return _gaZ;}

	void SetGravityX(double gaX) { _gaX = gaX;}
	void SetGravityY(double gaY) { _gaY = gaY;}
	void SetGravityZ(double gaZ) { _gaZ = gaZ;}

/*
    double GetXAccel() {return _xAccel;}
    double GetYAccel() {return _yAccel;}
    double GetZAccel() {return _zAccel;}

    void SetXAccel(double xAccel) {_xAccel = xAccel;}
    void SetYAccel(double yAccel) {_yAccel = yAccel;}
    void SetZAccel(double zAccel) {_zAccel = zAccel;}
*/
    double GetUVel() {return _uVel;}
    double GetVVel() {return _vVel;}
    double GetWVel() {return _wVel;}

    void SetUVel(double uVel) { _uVel = uVel;}
    void SetVVel(double vVel) { _vVel = vVel;}
    void SetWVel(double wVel) { _wVel = wVel;}
// Do we need angular acceleration?

//Get Quaternion Orientation
    double Getq0() {return _q0;}
    double Getq1() {return _q1;}
    double Getq2() {return _q2;}
    double Getq3() {return _q3;}

    void Setq0(double q0) {_q0 = q0;}
    void Setq1(double q1) {_q1 = q1;}
    void Setq2(double q2) {_q2 = q2;}
    void Setq3(double q3) {_q3 = q3;}

	void NormalizeQuat();

	double GetPhi() {return atan((2*_q0*_q1 + _q2*_q3)/(_q0*_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3));}
	double GetTheta() { return asin(2*(_q0*_q2 - _q1*_q3));}
	double GetPsi() {return atan((2*_q0*_q3 + _q1*_q2)/(_q0*_q0 + _q1*_q1 - _q2*_q2 - _q3*_q3));}

    double GetTime() {return _time;}
    double GetDTime() {return _dTime;}
    void SetTime(double time);

	double GetCompassData1() { return _Compass1;}
	double GetCompassData2() { return _Compass2;}
	double GetCompassData3() { return _Compass3;}

	void SetCompass1(double Compass1) { _Compass1 = Compass1;}
	void SetCompass2(double Compass2) { _Compass2 = Compass2;}
	void SetCompass3(double Compass3) { _Compass3 = Compass3;}

// started from a ball!
    void cgState(double time);

//    void const DisplayState();

//    void const WriteToFile();

/*
    const double GetThrust()const{return motor1Thrust;}   
    const double GetThrust()const{return motor2Thrust;}   
    const double GetThrust()const{return motor3Thrust;}   
    const double GetThrust()const{return motor4Thrust;}   
*/
  private:
// quadcopter state. might not need it
    bool on_Off;
    
  //  ofstream outputFile;

    double _mass;
// Angular-Velocity |Body  |raw
    double _pAngV;
    double _qAngV;
    double _rAngV;
/*
// Linear Accel     |Body  |raw
    double _xAccel;
    double _yAccel;
    double _zAccel;
*/
// Position         |Global|from linear vel
    double _xPos;
    double _yPos;
    double _zPos;
    
// Linear velocity  |Global|from accel
    double _dxPos;
    double _dyPos;
    double _dzPos;
    
// Linear velocity  |Body  |from accel
    double _uVel;
    double _vVel;
    double _wVel;

// Quaternion Orient|Global|from angular velocity  
    double _q0;
    double _q1;
    double _q2;
    double _q3;


    double _time;
    double _dTime;

	double _gaX;
	double _gaY;
	double _gaZ;

	double _Compass1;
	double _Compass2;
	double _Compass3;
    
// %thrust provided by motor
    double _motor1Thrust;
    double _motor2Thrust;
    double _motor3Thrust;
    double _motor4Thrust;
};

