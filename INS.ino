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
int16_t gyro[3];

Plotter p;
Plotter q;

    VectorXd TheState(15);
    int SampleSize = 200;
    VectorXd aXSample(SampleSize), aYSample(SampleSize), aZSample(SampleSize), gyrXSample(SampleSize), gyrYSample(SampleSize), gyrZSample(SampleSize);
    //MatrixXd Debugger(8,8);

    double ax, ay, az, dt=0.004;
    double baX = 0.0, baY = 0.0, baZ = 0.0, ba2X = 0.0, ba2Y = 0.0, ba2Z = 0.0; 
    double bgX = 0.0, bgY = 0.0, bgZ = 0.0, bg2X = 0.0, bg2Y = 0.0, bg2Z = 0.0;
    double ux, uy, uz, ugx, ugy, ugz; //measurement uncertainty
    double accelEps = 0.85;//1.3
    double gyrEps = .5;
    double gyrX, gyrY, gyrZ;
    double magX, magY, magZ;
    double time;
    double prevtime;
    double currtime;
    double g = 9.795;

    int serialOption = 6;

void setup(){
  Wire.begin();
  delay(100);
  Serial.begin(9600);
  delay(200);
  //IMU.calibrate(); //we got our own way
  IMU.init();
  q.AddTimeGraph("Raw Acceleration X", 15000, "ax", ax);
  q.AddTimeGraph("Raw Acceleration Y", 15000, "ay", ay);
  q.AddTimeGraph("Raw Acceleration Z", 15000, "ay", az);
  getSensorBias(); //comment out when you want raw data 
  delay(200);
//  p.AddXYGraph("Acceleration in X vs Custom Time", 1000000, "Time", time, "X-Acceleration");  //keep commented out
  p.AddTimeGraph("Accel vs Time", 2000, "ax", TheState(6), "ay", TheState(7), "az", TheState(8) );
  p.AddTimeGraph("Velocity vs Time", 2000, "u", TheState(3), "v", TheState(4), "w", TheState(5) );
  p.AddTimeGraph("Position vs Time", 2000, "x", TheState(0), "y", TheState(1), "z", TheState(2) );
  p.AddTimeGraph("Angular Velocity vs Time", 2000, "PhiDot", TheState(12), "ThetaDot", TheState(13), "PsiDot", TheState[14]);
  p.AddTimeGraph("Angle  vs Time", 2000, "phi", TheState(9), "theta", TheState(10), "psi", TheState(11) );
//  p.AddTimeGraph("Anglular Velocity vs Time", 15000, "r", Sdot[4] );  
//  p.AddTimeGraph("Angle vs Time", 15000, "psi", S[4] ); 
}

void loop(){
    //Init time
    time =0.0;
    getSensor(ax,ay,dt);
    currtime = millis()/1000.0;    
    Kalman Filter(dt, accelEps, gyrEps, ux, uy, uz, ugx, ugy, ugz); //think of somthing better for object name
    while(true){
      getSensor(ax,ay,dt);
      prevtime = currtime;
      currtime = millis()/1000.0;
      dt = currtime - prevtime;
      // Advance time
      time = time + dt;      
      
      //Filter
      Filter.UpdateState(dt, ax, ay, az, gyrX, gyrY, gyrZ); 
      TheState = Filter.KFilter();

      //No Filter
      //TheState = Filter.NoFilter(dt, ax, ay, gyrZ); //used when we want to compare... comment out above.
      
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
          case 4:
            Serial.print(time,3);
            Serial.print('\t');
            Serial.print (ax);
            Serial.print('\t');
            Serial.print (ay);
            Serial.print('\t');
            Serial.print(az);
            Serial.print('\t');
            Serial.print (gyrX);
            Serial.print('\t');
            Serial.print (gyrY);
            Serial.print('\t');
            Serial.println (gyrZ);  
          case 5:
            Serial.print(time,3);
            Serial.print('\t');
            Serial.print (accel[0]);
            Serial.print('\t');
            Serial.print (accel[1]);
            Serial.print('\t');
            Serial.print(accel[2]);
            Serial.print('\t');
            Serial.print (gyro[0]);
            Serial.print('\t');
            Serial.print (gyro[1]);
            Serial.print('\t');
            Serial.println (gyro[2]);  
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
void getSensor(double& ax, double& ay, double& dt){
    IMU.readAccelData(&accel[0]);
    IMU.readGyroData(&gyro[0]);
    
    double xScaled = (double) accel[0] * IMU.AccelRes;
    double yScaled = (double) accel[1] * IMU.AccelRes;
    double zScaled = (double) accel[2] * IMU.AccelRes;
    ax = xScaled * g - baX;
    ay = yScaled * g - baY;
    az = zScaled * g - baZ;

    gyrX = ((double)gyro[0] * IMU.GyroRes) - bgX;
    gyrY = ((double)gyro[1] * IMU.GyroRes) - bgY;
    gyrZ = ((double)gyro[2] * IMU.GyroRes) - bgZ;
}

void getSensorFirst(double& ax, double& ay, double& dt){
    IMU.readAccelData(&accel[0]);
    IMU.readGyroData(&gyro[0]);
    double xScaled = (double) accel[0] * IMU.AccelRes;
    double yScaled = (double) accel[1] * IMU.AccelRes;
    double zScaled = (double) accel[2] * IMU.AccelRes;
    ax = xScaled * g;
    ay = yScaled * g;
    az = zScaled * g;
        
    gyrX = (double)gyro[0] * IMU.GyroRes;
    gyrY = (double)gyro[1] * IMU.GyroRes;
    gyrZ = (double)gyro[2] * IMU.GyroRes;
    
}

void getSensorBias(){
    Serial.println("Determining Sensor Bias");
    baX = 0.0;
    baY = 0.0;
    baZ = 0.0;
    ba2X = 0.0;
    ba2Y = 0.0;
    ba2Z = 0.0;

    bgX = 0.0;
    bgY = 0.0;
    bgZ = 0.0;
    bg2X = 0.0;
    bg2Y = 0.0;
    bg2Z = 0.0;
       
    int intRange = 1500;
    double range = (double) intRange;
  for (int i = 0; i < intRange; i++){
    getSensorFirst(ax,ay, dt);
    if (serialOption == 6){
      q.Plot();
    } else{
    Serial.print(i);
    Serial.print('\t');
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    //Serial.println(az);
    Serial.print('\t');
    Serial.print(gyrX);
    Serial.print('\t');
    Serial.print(gyrY);
    Serial.print('\t');
    Serial.println(gyrZ);
    
    }
    baX += ax;
    baY += ay;
    baZ += az;
 //normalizing new gravity will not result in 9.81. we'll correct that later
    bgX += gyrX;
    bgY += gyrY;
    bgZ += gyrZ;
  }
  baX /= range;
  baY /= range;
  baZ /= range;

  bgX /= range;
  bgY /= range;
  bgZ /= range;
    

  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < intRange; i++){
      getSensor(ax,ay,dt);
      if (serialOption == 6){
        q.Plot();
      } else{
        Serial.print(i);
        Serial.print('\t');
        Serial.print(ax);
        Serial.print('\t');
        Serial.print(ay);
        Serial.print('\t');
    //Serial.println(az);
        Serial.print('\t');
        Serial.print(gyrX);
        Serial.print('\t');
        Serial.print(gyrY);
        Serial.print('\t');
        Serial.println(gyrZ);
      }
      ba2X += ax;
      ba2Y += ay;
      ba2Z += az;

      bg2X += gyrX;
      bg2Y += gyrY;
      bg2Z += gyrZ;
    }
    ba2X /= range;
    ba2Y /= range;
    ba2Z /= range;
    bg2X /= range;
    bg2Y /= range;
    bg2Z /= range;
  
    baX += ba2X;
    baY += ba2Y;
    baZ += ba2Z;
    bgX += bg2X;
    bgY += bg2Y;
    bgZ += bg2Z;
    
    ba2X = 0.0;
    ba2Y = 0.0;
    ba2Z = 0.0;
    bg2X = 0.0;
    bg2Y = 0.0;
    bg2Z = 0.0;
  }
  
  Serial.println("Calculating Sample Variance...");
  //Creates lists of values to calculate sample variance
  for( int i = 0; i < (SampleSize); i++)
  {
    getSensor(ax,ay,dt);
    aXSample(i) = ax;
    aYSample(i) = ay;
    aZSample(i) = az;
    gyrXSample(i) = gyrX;
    gyrYSample(i) = gyrY;
    gyrZSample(i) = gyrZ;
 //   printPrettyArray(aXSample);
    //Serial.println(i);
  }
  
  ux = Variance(aXSample); //accounting for uncertainty 0.04
  uy = Variance(aYSample); //accounting for uncertainty 0.03
  uz = Variance(aZSample);
  ugx = Variance(gyrXSample);
  ugy = Variance(gyrYSample);
  ugz = Variance(gyrZSample);

  Serial.println("Measurement Noise: ");
  Serial.println("ux      uy      uz      ugx     ugy     ugz     baX     baY     baZ      bgX     bgY     bgZ");
  Serial.print(ux);
  Serial.print('\t');
  Serial.print(uy);
  Serial.print('\t');
  Serial.print(uz);
  Serial.print('\t');
  Serial.print(ugx);
  Serial.print('\t');
  Serial.print(ugy);
  Serial.print('\t');
  Serial.print(ugz);
  Serial.print('\t');
  Serial.print(baX);
  Serial.print('\t');
  Serial.print(baY);
  Serial.print('\t');
  Serial.print(baZ);
  Serial.print('\t');
  Serial.print(bgX);
  Serial.print('\t');
  Serial.print(bgY);
  Serial.print('\t');
  Serial.println(bgZ);
  Serial.print('\n');
}

double Variance(VectorXd Vec) 
{
  double sum, squaredsum;
  sum = 0.0;
  squaredsum = 0.0;
  for (int i=0; i < Vec.size(); i++)
  {
    sum += Vec(i);
    squaredsum += (Vec(i)*Vec(i));
  }
  double average = sum/Vec.size();
  double ans = squaredsum/Vec.size() - (average*average);
/*  Serial.print(sum);
  Serial.print('\t');
  Serial.print(average);
  Serial.print('\t');
  Serial.println(ans,3);*/
  return squaredsum/Vec.size() - (average*average);
}

