// MPU-9250 Complementary Filter Example
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
int16_t accel[3];

float ax, ay, az, gx, gy, gz;
float pitch, roll, yaw;
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
    imu.readAccelData(&accel[0]);

    imu.Now = micros();
    imu.DeltaTime = ((imu.Now - imu.LastUpdate) / 1000000.0f);
    imu.LastUpdate = imu.Now;
    sum += imu.DeltaTime;
    sumCount++;

    imu.complementaryFilter(&accel[0], &gyro[0], &pitch, &roll, &yaw);

    if (abs(yaw) >= 360) yaw = 0;

    Serial.print("Roll: ");
    Serial.print((int16_t)roll);
    Serial.print("   Pitch: ");
    Serial.print((int16_t)pitch);
    Serial.print("   Yaw: ");
    Serial.println((int16_t)yaw);

    //Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    sumCount = 0;
    sum = 0;
}
