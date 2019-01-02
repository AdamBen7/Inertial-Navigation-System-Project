// MPU-9250 Accelerometer Example
// Jordan Day

/*
=========================================
This code is placed under the MIT license
Copyright (c) 2016 Jordan Day

Please reference the LICENSE file
=========================================
*/

#include "MPU9250.h"

MPU9250 imu;

int16_t accel[3];

void setup()
{
    Serial.begin(9600);
    imu.calibrate();
    imu.init();
}

void loop()
{
    float pitch, roll, fX, fY, fZ;

    imu.readAccelData(&accel[0]);

    fX = (float)accel[0] * imu.AccelRes;
    fY = (float)accel[1] * imu.AccelRes;
    fZ = (float)accel[2] * imu.AccelRes;

    // Print g force
    /*Serial.print("X: ");
    Serial.print(fX);
    Serial.print(" Y: ");
    Serial.print(fY);
    Serial.print(" Z: ");
    Serial.println(fZ);*/

    pitch = (atan2(fY, sqrt(fX * fX + fZ * fZ)) * 180.0) / M_PI;
    roll = (atan2(fX, sqrt(fY * fY + fZ * fZ)) * 180.0) / M_PI;

    Serial.print("Roll: ");
    Serial.print((int8_t)roll);
    Serial.print("  Pitch: ");
    Serial.println((int8_t)pitch);

    delay(50);
}
