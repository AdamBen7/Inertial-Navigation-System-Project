//initialize the values.
//These will be replaced by data from IMU

Ball::Ball: 
    ballMass(1.0),
    accelX(inAccelX),
    accelY(inAccelY),
    accelZ(inAccelZ){
}

double inAccelX;
double inAccelY; 
double inAccelZ ;

/*
double inPosX = 0;
double inPosY = 0;
double inPosZ = 0;
*/

double ballMass; //kg


void Ball::SetAccelX(const double inAccelX){
  accelX = inAccelX;
}

void Ball::SetAccelY(const double inAccelY){
  accelY = inAccelY;
}
 
void Ball::SetAccelZ(const double inAccelZ){
  accelX = inAccelZ;
}

void Ball::SetVel(const double inAccelX, inAccelY, inAccelZ){
  velX += .5*accelX; //time?
  velY += .5*accelY;
  velZ += .5*accelZ;
}

void Ball::SetPos(const double velX, velY, velZ){
  posX += velX; //time?
  posY += velY;
  posZ += velZ;
}

double sumForceX = inAccelX*ballMass;
double sumForceY = inAccelY*ballMass;
double sumForceZ = inAccelZ*ballMass;

/*
void Ball::SetPosX(const double inPosX){
  posX = inPosX;
}

void Ball::SetPosY(const double inPosY){
  posY = inPosY;
}

void Ball::SetPosZ(const double inPosZ){
  posZ = inPosZ;
}
*/

void Ball::SetForceX(const double sumForceX){
  forceX = sumForceX;
}

void Ball::SetForceY(const double sumForceY){
  forceY = sumForceY;
}

void Ball::SetForceZ(const double sumForceZ){
  forceZ = sumForceZ;
}


