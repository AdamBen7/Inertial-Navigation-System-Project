
//class Motor;

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

    double GetPAngVel() {return _pAngV;}
    double GetQAngVel() {return _qAngV;}
    double GetRAngVel() {return _rAngV;}
   
    void SetPAngVel(double pAngV) { _pAngV = pAngV;}
    void SetQAngVel(double qAngV) { _qAngV = qAngV;}
    void SetRAngVel(double rAngV) { _rAngV = rAngV;}

    double GetXAccel() {return _xAccel;}
    double GetYAccel() {return _yAccel;}
    double GetZAccel() {return _zAccel;}

    void SetXAccel(double xAccel) {_xAccel = xAccel;}
    void SetYAccel(double yAccel) {_yAccel = yAccel;}
    void SetZAccel(double zAccel) {_zAccel = zAccel;}

/*
    double GetUOrr() {return _uOrr;}
    double GetVOrr() {return _vOrr;}
    double GetWOrr() {return _wOrr;}
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
    double Getq2() {return _q1;}
    double Getq3() {return _q1;}

    void Setq0(double q0) {_q0 = q0;}
    void Setq1(double q1) {_q1 = q1;}
    void Setq2(double q2) {_q2 = q2;}
    void Setq3(double q3) {_q3 = q3;}

// started from a ball!
    void cgState(double time);

    void const DisplayState();

/*
    const double GetThrust()const{return motor1Thrust;}   
    const double GetThrust()const{return motor2Thrust;}   
    const double GetThrust()const{return motor3Thrust;}   
    const double GetThrust()const{return motor4Thrust;}   
*/
  private:
// quadcopter state. might not need it
    bool on_Off;

    double _mass;
// Angular-Velocity |Body  |raw
    double _pAngV;
    double _qAngV;
    double _rAngV;
// Linear Accel     |Body  |raw
    double _xAccel;
    double _yAccel;
    double _zAccel;

// Position         |Global|from linear vel
    double _xPos;
    double _yPos;
    double _zPos;
    
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
    
// %thrust provided by motor
    double _motor1Thrust;
    double _motor2Thrust;
    double _motor3Thrust;
    double _motor4Thrust;
};

