
class Motor;

class Quadcopter
{

  public:
    Quadcopter();
    ~Quadcopter();

    double GetXPos() {return _xPos;}
    double GetYPos() {return _yPos;}
    double GetZPos() {return _zPos;}

    double GetXVel() {return _xVel;}
    double GetYVel() {return _yVel;}
    double GetZVel() {return _zVel;}

    double GetXAccel() {return _xAccel;}
    double GetYAccel() {return _yAccel;}
    double GetZAccel() {return _zAccel;}

    double GetUOrr() {return _uOrr;}
    double GetVOrr() {return _vOrr;}
    double GetWOrr() {return _wOrr;}

    double GetUVel() {return _uVel;}
    double GetVVel() {return _vVel;}
    double GetWVel() {return _wVel;}

// Do we need angular acceleration?

    void SetXAccel(double xAccel) {_xAccel = xAccel;}
    void SetYAccel(double yAccel) {_yAccel = yAccel;}
    void SetZAccel(double zAccel) {_zAccel = zAccel;}

// started from a ball!
    void cgState(double time);

    const double GetThrust()const{return motor1Thrust;}   
    const double GetThrust()const{return motor2Thrust;}   
    const double GetThrust()const{return motor3Thrust;}   
    const double GetThrust()const{return motor4Thrust;}   

  private:
// quadcopter state. might not need it
    bool on-off;

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


    double _Time;
    
// %thrust provided by motor
    double _motor1Thrust;
    double _motor2Thrust;
    double _motor3Thrust;
    double _motor4Thrust;
};

