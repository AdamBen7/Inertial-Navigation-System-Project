

class Ball{

  public:
    Ball();
    ~Ball();
    void SetPos(const float velX, velY, velZ);
    void SetVel(const float accelX, accelY, accelZ);


//Do i need this? Can just put in constructor
    void SetAccelX(const float inForceX);


/*
    void SetPosX(const float inPosX);
    void SetPosY(const float inPosY);
    void SetPosZ(const float inPosZ);
*/
    void SetForceX(const float inForceX);
    void SetForceY(const float inForceY);
    void SetForceZ(const float inForceZ);

  private:
    const float ballMass;

// position with respect to origin
    float posX;
    float posY;
    float posZ;

    float velX;
    float velY;
    float velZ;
    
    float accelX;
    float accelY;
    float accelZ;

//experienced force. Unnecessary?
    float forceX;
    float forceY;
    float forceZ;

    int time; //millis function sets this?
};

