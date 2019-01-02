#include "Arduino.h"
#include <Wire.h>

#ifndef GY_85_h
#define GY_85_h

//----------addresses----------//
#define ADXL345 (0x53)         // Device address as specified in data sheet //ADXL345 accelerometer
#define DATAX0  (0x32)         //X-Axis Data 0
//#define DATAX1 0x33          //X-Axis Data 1
//#define DATAY0 0x34          //Y-Axis Data 0
//#define DATAY1 0x35          //Y-Axis Data 1
//#define DATAZ0 0x36          //Z-Axis Data 0
//#define DATAZ1 0x37          //Z-Axis Data 1
#define HMC5883 (0x1E)         //gyro
#define ITG3200 (0x68)         //compass


class GY_85
{
    
private:
    void GyroCalibrate();
    void SetGyro();
    void SetCompass();
    void SetAccelerometer();
    
public:
    void   init();
    short int*   readFromAccelerometer();
    short int*   readFromCompass();
    float* readGyro();
    
    //callback functions
    inline short int accelerometer_x( short int* a ){ return *(   a ); }
    inline short int accelerometer_y( short int* a ){ return *( 1+a ); }
    inline short int accelerometer_z( short int* a ){ return *( 2+a ); }
    
    //-----------------------------------
    
    inline short int compass_x( short int* a ){ return *(   a ); }
    inline short int compass_y( short int* a ){ return *( 1+a ); }
    inline short int compass_z( short int* a ){ return *( 2+a ); }
    
    //-----------------------------------
    
    //Divide by 14.375 to get degrees/sec
    //Divide by 823.62683050057 to get rad/sec
    inline float gyro_x( float* a ){ return *(   a ) / 823.62683050057; }
    inline float gyro_y( float* a ){ return *( 1+a ) / 823.62683050057; }
    inline float gyro_z( float* a ){ return *( 2+a ) / 823.62683050057; }
    inline float temp  ( float* a ){ return  35+( *( 3+a )+13200 ) / 280; }
};

#endif
