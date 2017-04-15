// MPU-9250 Gyro Angles Example
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

int16_t gyro[3];
float fX, fY, fZ, angleX, angleY, angleZ;
uint32_t sumCount = 0;
float sum = 0.0f;

void setup()
{
    Serial.begin(115200);
    imu.calibrate();
    imu.init();
}

void loop()
{
    imu.readGyroData(&gyro[0]);

    // Convert raw data to degrees per second
    fX = (float)gyro[0] * imu.GyroRes;
    fY = (float)gyro[1] * imu.GyroRes;
    fZ = (float)gyro[2] * imu.GyroRes;

    imu.Now = micros();
    imu.DeltaTime = ((imu.Now - imu.LastUpdate) / 1000000.0f);
    imu.LastUpdate = imu.Now;
    sum += imu.DeltaTime;
    sumCount++;

    angleX += fX * imu.DeltaTime;
    angleY += fY * imu.DeltaTime;
    angleZ += fZ * imu.DeltaTime;

    Serial.print("Pitch: ");
    Serial.print((int8_t)angleX);
    Serial.print(" Roll: ");
    Serial.print((int8_t)angleY);
    Serial.print(" Yaw: ");
    Serial.println((int8_t)angleZ);

    //Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    sumCount = 0;
    sum = 0;
}
