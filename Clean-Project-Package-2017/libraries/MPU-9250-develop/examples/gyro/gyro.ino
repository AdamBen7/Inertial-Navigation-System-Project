// MPU-9250 Gyro Example
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
float fX, fY, fZ;

void setup()
{
    Serial.begin(9600);
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

    Serial.print("Pitch:");
    if (fX >= 0)
        Serial.print("+");
    Serial.print(fX, 0);

    if (fX - 2 > 0)
        Serial.print(" NoU\t");
    else if (fX + 2 < 0)
        Serial.print(" NoD\t");
    else
        Serial.print(" ---\t");

    // Display roll in degrees per second
    Serial.print("  Roll:");
    if (fY >= 0)
        Serial.print("+");

    Serial.print(fY, 0);

    if (fY - 2 > 0)
        Serial.print(" RwD\t");
    else if (fY + 2 < 0)
        Serial.print(" RwU\t");
    else
        Serial.print(" ---\t");

    // Display yaw in degrees per second
    Serial.print("  Yaw:");
    if (fZ >= 0)
        Serial.print("+");

    Serial.print(fZ, 0);

    if (fZ - 2 > 0)
        Serial.println(" NoR\t");
    else if (fZ + 2 < 0)
        Serial.println(" NoL\t");
    else
        Serial.println(" ---\t");

    delay(50);
}
