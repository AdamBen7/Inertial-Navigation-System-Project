//#include <iostream>
#include <cmath>
#include <Wire.h>
#include "MPU9250.h"
#include "Plotter.h"
#include "Eigen30.h"
#include <Eigen/LU>
#include "Kalman.h"

MPU9250 IMU;
int16_t accel[3];

Plotter p;
Plotter q;

    VectorXd TheState(6);
    MatrixXd Debugger(6,6);

    double ax, ay, az, r, dt=0.004; //0.004 changed since didn't look right
    double baX, baY, baZ, ba2X, ba2Y, ba2Z; 
    double maxaX, minaX, maxaY, minaY;
    double ux, uy;
    double epsilon = 1.3;
    double gyrX, gyrY, gyrZ;
    double magX, magY, magZ;
    double time;
    double prevtime;
    double currtime;
    double g = 9.795;

    int serialOption = 6;

void setup(){
  Wire.begin();
  delay(200);
  Serial.begin(9600);
  delay(500);
  //IMU.calibrate(); //we got our own way
  IMU.init();
  q.AddTimeGraph("Raw Acceleration X", 15000, "ax", ax);
  q.AddTimeGraph("Raw Acceleration Y", 15000, "ay", ay);
  getSensorBias();
  delay(500);
//  p.AddXYGraph("Acceleration in X vs Custom Time", 1000000, "Time", time, "X-Acceleration");  //keep commented out
  p.AddTimeGraph("Accel vs Time", 2000, "ax", TheState(4), "ay", TheState(5));
  p.AddTimeGraph("Velocity vs Time", 2000, "u", TheState[2], "v", TheState[3] );
  p.AddTimeGraph("Position vs Time", 2000, "x", TheState[0], "y", TheState[1] );
//  p.AddTimeGraph("Anglular Velocity vs Time", 15000, "r", Sdot[4] );  
//  p.AddTimeGraph("Angle vs Time", 15000, "psi", S[4] ); 
}

void loop(){
    //Init time
    time =0.0;
    getSensor(ax,ay,r,dt);
    currtime = millis()/1000.0; 
    Kalman Filter(dt, epsilon, ux, uy); //think of somthing better for object name
    while(true){
      getSensor(ax,ay,r,dt);
      prevtime = currtime;
      currtime = millis()/1000.0;
      dt = currtime - prevtime;
      // Advance time
      time = time + dt;      
      
      //Filter
      Filter.UpdateState(dt, ax, ay); 
      TheState = Filter.KFilter();
      
      //No Filter
      //TheState = Filter.NoFilter(dt, ax, ay); //used when we want to compare... comment out above.
      
      //Debugger = Filter.Debugger();

      // this is for debugging on your serial plotter
      // 0 - skip 
      // 1 - plot gyro values
      // 2 - plot accelerometer values
      // 3 - plot magnetometer values
      // 4 - INS Table
      // 5 - Accel, Vel, Pos
      if (serialOption!=0) {
        //Serial.print (time,3);
        //Serial.print('\t');
        switch(serialOption) {
            
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

          case 6:
            p.Plot();
          break;

          case 7:
            Serial.print(time,3);
            Serial.print('\t');
            printPrettyArray(TheState);
            //printArray(Debugger);
          break;
        }
      }

    /*if(time > 60.0)
    {
      while(true){}
    }  */
  }

}

//Polymorphic Printing! Woohoo!
void printArray(Eigen::MatrixXd Mat){
  for(size_t i=0; i<Mat.rows(); i++){
    for (size_t j=0; j<Mat.cols(); j++){
      Serial.print(Mat(i,j),3);
      Serial.print('\t');
      }
    Serial.print('\n');
    } 
Serial.print('\n');
}

void printArray(Eigen::VectorXd Vec){

  for (size_t j=0; j<(Vec.size()); j++){
    Serial.print(Vec(j),3);
    Serial.print('\t');
    Serial.print('\n');
    } 
Serial.print('\n');
}

void printArray(double Vec[], size_t n){
    for(size_t i=0; i<n; i++){
//        cout << Vec[i] << " ";
    Serial.print(Vec[i],3);
    Serial.print('\t');
    }
Serial.print('\n');
//    cout << endl;
}

void printPrettyArray(Eigen::VectorXd Vec){

  for (size_t j=0; j<(Vec.size()); j++){
    Serial.print(Vec(j),3);
    Serial.print('\t');
    //Serial.print('\n'); //just because it displays nicely
    } 
Serial.print('\n');
}


//ax =1, ay = -1;
void getSensor(double& ax, double& ay, double& r, double& dt){
    IMU.readAccelData(&accel[0]);
    double xScaled = (double) accel[0] * IMU.AccelRes;
    double yScaled = (double) accel[1] * IMU.AccelRes;
    ax = xScaled * g - baX;
    ay = yScaled * g - baY;
    r = 0.0;
//    r = IMU.gyro_z( IMU.readGyro() );
}

void getSensorFirst(double& ax, double& ay, double& r, double& dt){
    IMU.readAccelData(&accel[0]);
    double xScaled = (double) accel[0] * IMU.AccelRes;
    double yScaled = (double) accel[1] * IMU.AccelRes;
    ax = xScaled * g;
    ay = yScaled * g;
    r = 0.0;
}

void getSensorBias(){
    baX = 0.0;
    baY = 0.0;
    ba2X = 0.0;
    ba2Y = 0.0;
    int intRange = 1500;
    double range = (double) intRange;
  for (int i = 0; i < intRange; i++){
    getSensorFirst(ax,ay,r,dt);
    if (serialOption == 6){
      q.Plot();
    } else{
    Serial.print(i);
    Serial.print('\t');
    Serial.print(ax);
    Serial.print('\t');
    Serial.println(ay);
    }
    baX += ax;
    baY += ay;
//    baZ += az; //normalizing new gravity will not result in 9.81. we'll correct that later
  }
  baX /= range;
  baY /= range;
  baZ /= range;  

  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < intRange; i++){
      getSensor(ax,ay,r,dt);
      if (serialOption == 6){
        q.Plot();
      } else{
      Serial.print(i);
      Serial.print('\t');
      Serial.print(ax);
      Serial.print('\t');
      Serial.println(ay);
      }
      ba2X += ax;
      ba2Y += ay;
    }
    ba2X /= range;
    ba2Y /= range;
    baX += ba2X;
    baY += ba2Y;
    ba2X = 0.0;
    ba2Y = 0.0;
  }

  maxaX = 0.0;
  minaX = 0.0;
  maxaY = 0.0;
  minaY = 0.0;

  for (int i = 0; i < intRange; i++)
  {
    getSensor(ax,ay,r,dt);
    if(ax > maxaX)
    {
      maxaX = ax;
    }
    if(ax < minaX)
    {
      minaX = ax;
    }
    if(ay > maxaY)
    {
      maxaY = ay;
    }
    if(ay < minaY)
    {
      minaY = ay;
    }
  }

  Serial.print(maxaX);
  Serial.print('\t');
  Serial.print(minaX);
  Serial.print('\t');
  Serial.print(maxaY);
  Serial.print('\t');
  Serial.println(minaY);
  ux = (maxaX > -minaX) ? (maxaX) : (-minaX);
  uy = (maxaY > -minaY) ? (maxaY) : (-minaY);

  ux += 0.04; //accounting for uncertainty
  uy += 0.03; //accounting for uncertainty

  Serial.print(ux);
  Serial.print('\t');
  Serial.print(uy);
  Serial.print('\t');
  Serial.print(baX);
  Serial.print('\t');
  Serial.println(baY);

}
