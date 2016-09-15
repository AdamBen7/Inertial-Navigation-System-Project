

class Ball{

  public:
    Ball();
    ~Ball();

    void SetPosX(const float inPosX);
    void SetPosY(const float inPosY);
    void SetPosZ(const float inPosZ);

    void SetForceX(const float inForceX);
    void SetForceY(const float inForceY);
    void SetForceZ(const float inForceZ);

  private:
    const float ballMass;

// position with respect to origin
    float posX;
    float posY;
    float posZ;
//experienced force
    float forceX;
    float forceY;
    float forceZ;

};

