//#include <iostream>
#include <cmath>
#include <Wire.h>
#include "GY_85.h"

GY_85 GY85;
//using namespace std;

    double S[5], Sdot[5], Sdot1[5];
    double ax, ay, az, r, dt;
    double baX, baY, baZ; 
    double gyrX, gyrY, gyrZ;
    double magX, magY, magZ;
    double time;
    double prevtime;
    double currtime;
    

void setup(){
  Wire.begin();
  delay(10);
  Serial.begin(9600);
  delay(10);
  GY85.init();
  getSensorBias(baX, baY);
  delay(2000);
}

void loop(){
    zeroArray(S, 5);
    zeroArray(Sdot, 5);
    zeroArray(Sdot1, 5);

    //Init time
    time =0.0;
    getSensor(ax,ay,r,dt);
    getSdot(Sdot1, S, ax,ay,r);

    currtime = millis()/1000.0; 

    while(true){
        // Read sensor data
        getSensor(ax,ay,r,dt); //work on efficiency since dt is unecessarily set to 0.1 everytime!

        // Advance time
        prevtime = currtime;
        currtime = millis()/1000.0;
        dt = currtime - prevtime;
        time = time + dt;

        // this is for debugging on your serial plotter (turn off if not needed)
        // 0 - skip 
        // 1 - plot gyro values
        // 2 - plot accelerometer values
        // 3 - plot magnetometer values
        // 4 - INS Table
        int plotOption = 4;

        if (plotOption!=0) {
          Serial.print (currtime,3);
          Serial.print('\t');
          switch(plotOption) {
            
            case 1:
              Serial.print (gyrX);
              Serial.print('\t');
              Serial.print (gyrY);
              Serial.print('\t');
              Serial.println (gyrZ);  
              break;

            case 2:
              Serial.print (ax);
              Serial.print('\t');
              Serial.print (ay);
              Serial.print('\t');
              Serial.println (az);
              break;
              
            case 3:
              Serial.print (magX);
              Serial.print('\t');
              Serial.print (magY);
              Serial.print('\t');
              Serial.println (magZ);
              break;

            case 4:        
            // Compute time derivative of states
            getSdot(Sdot, S, ax,ay,r);
            // Time advancement of states
            ExplicitEuler(S,dt,Sdot);
            //cout << time  << " ";
            Serial.print(time,3);
            Serial.print('\t');
            printArray(S, 5);
            break;  
            
            case 5:
              Serial.print (magX);
            
          }
        }

        
    }
}

void zeroArray(double Vec[], size_t n){
    for(size_t i=0; i<n; i++){
        Vec[i] = 0.0;
    }
}

void printArray(double Vec[], size_t n){
    for(size_t i=0; i<n; i++){
//        cout << Vec[i] << " ";
    Serial.print(Vec[i]);
    Serial.print('\t');
    }
Serial.print('\n');
//    cout << endl;
}
//ax =1, ay = -1;
void getSensor(double& ax, double& ay, double& r, double& dt){
    ax = GY85.accelerometer_x(GY85.readFromAccelerometer());
    ay = GY85.accelerometer_y(GY85.readFromAccelerometer());
    double xScaled = map(ax, -254.27, 263.98, -1000.0, 1000.0); //we might want to do this again.
    double yScaled = map(ay, -247.57, 270.24, -1000.0, 1000.0);
    //long zScaled = map(az, -248.7, 248.56, -1000, 1000);

    // re-scale to m/s^2
    ax = xScaled * .00981 - baX;
    ay = yScaled * .00981 - baY;
 // double zAccel = zScaled * .00981;

    //ax = 1;//ay = -0.0;//dt = 0.01;
    r = 0.0;
}

void getSensorBias(double& baX, double& baY){
    int range = 1000;
  for (int i = 0; i < 1000; i++){
    getSensor(ax,ay,r,dt);
    baX += ax;
    baY += ay;
//    baZ += az; //normalizing new gravity will not result in 9.81. we'll correct that later
  }
    
  baX /= range;
  baY /= range;
  baZ /= range;  
}

void Multistep2ptAdams(double y[], double timestep,
                       double ydot[], double ydotold[]){
    // Warning: this directly updates y
    for(size_t i=0; i<5; i++){
        y[i] = y[i] + timestep*(1.5*ydot[i]-0.5*ydotold[i]);
    }
}
void ExplicitEuler(double y[], double timestep,
                       double ydot[]){
    // Warning: this directly updates y
    for(size_t i=0; i<5; i++){
        y[i] = y[i] + timestep*ydot[i];
    }
}

void getSdot(double Sdot[], double State[], double ax, double ay, double r){
    double x = State[0];
    double y = State[1];
    double u = State[2];
    double v = State[3];
    double psi = State[4];
    // Calc
    Sdot[0] = cos(psi)*u - sin(psi)*v;
    Sdot[1] = sin(psi)*u + cos(psi)*v;
    Sdot[2] = ax;
    Sdot[3] = ay;
    Sdot[4] = r;
}
