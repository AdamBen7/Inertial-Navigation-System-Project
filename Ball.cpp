//initialize the values.
//These will be replaced by data from IMU

float sumForceX;
float sumForceY;
float sumForceZ;

float inPosX;
float inPosY;
float inPosZ;

float ballMass; //kg

Ball::Ball: 
    ballMass(1.0),
    posX(inPosX),
    posY(inPosY),
    posZ(inPosZ){
}

void Ball::SetPosX(const float inPosX){
  posX = inPosX;
}

void Ball::SetPosY(const float inPosY){
  posY = inPosY;
}

void Ball::SetPosZ(const float inPosZ){
  posZ = inPosZ;
}

void Ball::SetForceX(const float sumForceX){
  forceX = sumForceX;
}

void Ball::SetForceY(const float sumForceY){
  forceY = sumForceY;
}

void Ball::SetForceZ(const float sumForceZ){
  forceZ = sumForceZ;
}


inAccelX = sumForceX/ballMass
inAccelY = sumForceY/ballMass

 
void Ball::SetVelocity(const float 


Get
float
float
float
