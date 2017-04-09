//#include <iostream>
#include <cmath>
#include <Wire.h>
#include "GY_85.h"
#include "Plotter.h"
#include "Eigen30.h"


GY_85 GY85;
Plotter p;
Plotter q;

    double S[5], Sdot[5], Sdot1[5];
    double ax, ay, az, r, dt;
    double baX, baY, baZ, ba2X, ba2Y, ba2Z; 
    double maxaX, minaX, maxaY, minaY;
    double ux, uy;
    double gyrX, gyrY, gyrZ;
    double magX, magY, magZ;
    double time;
    double prevtime;
    double currtime;
    double g = 9.795;
    
    int plotOption = 5;

void setup(){
  Wire.begin();
  delay(1000);
  Serial.begin(9600);
  delay(1000);
  GY85.init();
  q.AddTimeGraph("Raw Acceleration X", 15000, "ax", ax);
  q.AddTimeGraph("Raw Acceleration Y", 15000, "ay", ay);
  getSensorBias();
  delay(1000);
//  p.AddXYGraph("Acceleration in X vs Custom Time", 1000000, "Time", time, "X-Acceleration"); 
  p.AddTimeGraph("Accel vs Time", 15000, "ax", ax, "ay", ay );
  p.AddTimeGraph("Velocity vs Time", 15000, "u", S[2], "v", S[3] );
  p.AddTimeGraph("Position vs Time", 15000, "x", S[0], "y", S[1] );
  p.AddTimeGraph("Anglular Velocity vs Time", 15000, "r", Sdot[4] );  
  p.AddTimeGraph("Angle vs Time", 15000, "psi", S[4] );  
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
      //Serial.println("Top of loop...");
      while(millis() % 4 != 0){}
        // Read sensor data
        getSensor(ax,ay,r,dt); //work on efficiency since dt is unecessarily set to 0.1 everytime!
        // Advance time
        /*
        prevtime = currtime;
        currtime = millis()/1000.0;
        dt = currtime - prevtime;
        */
        dt = .004;
        time = time + dt;
        // Compute time derivative of states
        getSdot(Sdot, S, ax,ay,r);
        // Time advancement of states
        ExplicitEuler(S,dt,Sdot);           

        // this is for debugging on your serial plotter (turn off if not needed)
        // 0 - skip 
        // 1 - plot gyro values
        // 2 - plot accelerometer values
        // 3 - plot magnetometer values
        // 4 - INS Table
        // 5 - Accel, Vel, Pos
        
        if (plotOption!=0) {
          //Serial.print (time,3);
          //Serial.print('\t');
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
            //cout << time  << " ";
            Serial.print(time,3);
            Serial.print('\t');
            printArray(S, 5);
            break;  
            
            case 5:
              Serial.print(millis());
              Serial.print('\t');
              Serial.print(time,3);
              Serial.print('\t');
              Serial.print (ax);
              Serial.print('\t');
              Serial.print (ay);
              Serial.print('\t');
              //Serial.print (az);
              //Serial.print('\t');
            printArray(S, 5);
            break;  

            case 6:
              p.Plot();
            break;
          }
        }
/*
    if(time > 60.0)
    {
      while(true){}
    }  */
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
    short shortax = GY85.accelerometer_x(GY85.readFromAccelerometer());
    short shortay = GY85.accelerometer_y(GY85.readFromAccelerometer());
 
    double xScaled = (double) map(shortax, -254.27, 263.7, -1000, 1000);
    double yScaled = (double) map(shortay, -248.62, 270.05, -1000, 1000);

    //long zScaled = map(az, -248.7, 248.56, -1000, 1000);

    // re-scale to m/s^2
    ax = (xScaled *  g/1000.0) - baX;
    ay = (yScaled *  g/1000.0) - baY;
    
 // double zAccel = zScaled * .00981;

    //ax = 1;//ay = -0.0;//dt = 0.01;
    r = GY85.gyro_z( GY85.readGyro() );
}

void getSensorFirst(double& ax, double& ay, double& r, double& dt){
    short shortax = GY85.accelerometer_x(GY85.readFromAccelerometer());
    short shortay = GY85.accelerometer_y(GY85.readFromAccelerometer());

    double xScaled = (double) map(shortax, -254.27, 263.7, -1000, 1000);
    double yScaled = (double) map(shortay, -248.62, 270.05, -1000, 1000);
    //long zScaled = map(az, -248.7, 248.56, -1000, 1000);
    // re-scale to m/s^2
    ax = xScaled * g/1000.0;
    ay = yScaled * g/1000.0;
 // double zAccel = zScaled * .00981;

    //ax = 1;//ay = -0.0;//dt = 0.01;
    r = 0.0;
}

void getSensorBias(){
    baX = 0.0;
    baY = 0.0;
    ba2X = 0.0;
    ba2Y = 0.0;
    int intRange = 2000;
    double range = (double) intRange;
  for (int i = 0; i < intRange; i++){
    getSensorFirst(ax,ay,r,dt);
    if (plotOption == 6){
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

  for (int j = 0; j < 4; j++)
  {
    for (int i = 0; i < intRange; i++){
      getSensor(ax,ay,r,dt);
      if (plotOption == 6){
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

  Serial.print(ux);
  Serial.print('\t');
  Serial.print(uy);
  Serial.print('\t');
  Serial.print(baX);
  Serial.print('\t');
  Serial.println(baY);

}
/* //This didn't improve anything... thought it was a good idea to get 9.81 as reading when accelerometer pointed at ground.
void getSensor(double& ax, double& ay, double& r, double& dt){
    short shortax = GY85.accelerometer_x(GY85.readFromAccelerometer());
    short shortay = GY85.accelerometer_y(GY85.readFromAccelerometer());
 
    double xScaled = (double) map(shortax, -254.27, 263.7, -1000, 1000); //we might want to do this again.
    double yScaled = (double) map(shortay, -248.62, 270.05, -1000, 1000);

    xScaled = xScaled;
    yScaled = yScaled;
    //long zScaled = map(az, -248.7, 248.56, -1000, 1000);

    // re-scale to m/s^2
    ax = xScaled *  g - baX;
    ay = yScaled *  g - baY;

    ax = map(ax, -(1000*g + baX), (1000*g + baX), -1000*g, 1000*g);
    ay = map(ay, -(1000*g + baY), (1000*g + baY), -1000*g, 1000*g);

    ax /= 1000;
    ay /= 1000;
 // double zAccel = zScaled * .00981;

    //ax = 1;//ay = -0.0;//dt = 0.01;
    r = 0.0;
}

void getSensorFirst(double& ax, double& ay, double& r, double& dt){
    short shortax = GY85.accelerometer_x(GY85.readFromAccelerometer());
    short shortay = GY85.accelerometer_y(GY85.readFromAccelerometer());

    double xScaled = (double) map(shortax, -254.27, 263.7, -1000, 1000); //we might want to do this again.
    double yScaled = (double) map(shortay, -248.62, 270.05, -1000, 1000);
    //long zScaled = map(az, -248.7, 248.56, -1000, 1000);
    xScaled = xScaled;
    yScaled = yScaled;
    // re-scale to m/s^2
    ax = xScaled * g;
    ay = yScaled * g;
 // double zAccel = zScaled * .00981;

    //ax = 1;//ay = -0.0;//dt = 0.01;
    r = 0.0;
}*/



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
