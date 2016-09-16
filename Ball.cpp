//initialize the values.
//These will be replaced by data from IMU

Ball::Ball: 
    ballMass(1.0),
    posX(inPosX),
    posY(inPosY),
    posZ(inPosZ){
}

float inAccelX;
float inAccelY; 
float inAccelZ ;

float inPosX = 0;
float inPosY = 0;
float inPosZ = 0;

float ballMass; //kg


void Ball::SetAccelX(const float inAccelX){
  accelX = inAccelX;
}

void Ball::SetAccelY(const float inAccelY){
  accelY = inAccelY;
}
 
void Ball::SetAccelZ(const float inAccelZ){
  accelX = inAccelZ;
}

void Ball::SetVel(const float inAccelX, inAccelY, inAccelZ){
  velX += .5*accelX; //time?
  velY += .5*accelY;
  velZ += .5*accelZ;
}

void Ball::SetPos(const float velX, velY, velZ){
  posX += velX; //time?
  posY += velY;
  posZ += velZ;
}

float sumForceX = inAccelX*ballMass;
float sumForceY = inAccelY*ballMass;
float sumForceZ = inAccelZ*ballMass;

/*
void Ball::SetPosX(const float inPosX){
  posX = inPosX;
}

void Ball::SetPosY(const float inPosY){
  posY = inPosY;
}

void Ball::SetPosZ(const float inPosZ){
  posZ = inPosZ;
}
*/

void Ball::SetForceX(const float sumForceX){
  forceX = sumForceX;
}

void Ball::SetForceY(const float sumForceY){
  forceY = sumForceY;
}

void Ball::SetForceZ(const float sumForceZ){
  forceZ = sumForceZ;
}


