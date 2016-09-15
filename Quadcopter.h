
class Motor;

class Quadcopter
{

  public:
    Quadcopter();
    ~Quadcopter();

    const float GetThrust()const{return motor1Thrust;}   
    const float GetThrust()const{return motor2Thrust;}   
    const float GetThrust()const{return motor3Thrust;}   
    const float GetThrust()const{return motor4Thrust;}   


  private:
// quadcopter state. might not need it
    bool on-off;

// position of quadcopter
    float xPos;
    float yPos;
    float zPos;
    
// orientation of quadcopter
    float uOrr;
    float vOrr;
    float wOrr;

// %thrust provided by motor
    float motor1Thrust;
    float motor2Thrust;
    float motor3Thrust;
    float motor4Thrust;
};

model
