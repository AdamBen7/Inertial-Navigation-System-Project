
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

// position of quadcopter
    double _xPos;
    double _yPos;
    double _zPos;
    
    double _xVel;
    double _yVel;
    double _zVel;

//Accelerometer feeds us this data
    double _xAccel;
    double _yAccel;
    double _zAccel;

// orientation of quadcopter
    double _uOrr;
    double _vOrr;
    double _wOrr;

// Gyroscope feeds us this data
    double _uVel;
    double _vVel;
    double _wVel;

    double _Time;
    
// %thrust provided by motor
    double _motor1Thrust;
    double _motor2Thrust;
    double _motor3Thrust;
    double _motor4Thrust;
};

