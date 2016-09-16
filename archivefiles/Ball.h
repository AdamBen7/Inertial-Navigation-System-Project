

class Ball{

  public:
    Ball();
    ~Ball();
    void SetPos(const double velX, velY, velZ);
    void SetVel(const double accelX, accelY, accelZ);


//Do i need this? Can just put in constructor
    void SetAccelX(const double inForceX);


/*
    void SetPosX(const double inPosX);
    void SetPosY(const double inPosY);
    void SetPosZ(const double inPosZ);
*/
    void SetForceX(const double inForceX);
    void SetForceY(const double inForceY);
    void SetForceZ(const double inForceZ);

  private:
    double ballMass;

// position with respect to origin
    double posX;
    double posY;
    double posZ;

    double velX;
    double velY;
    double velZ;
    
    double accelX;
    double accelY;
    double accelZ;

//experienced force. Unnecessary?
    double forceX;
    double forceY;
    double forceZ;

    int time; //millis function sets this?
};

