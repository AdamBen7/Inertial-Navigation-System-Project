// MPU-9250 I2C device class
// Based on InvenSense MPU-9250 Register Map and Descriptions Rev. 1.4, 9/9/2013 (RM-MPU-9250A-00)
// Jordan Day

/*
=========================================
This code is placed under the MIT license
Copyright (c) 2016 Jordan Day

Please reference the LICENSE file
=========================================
*/

#include "MPU9250.h"

MPU9250::MPU9250()
{
    Wire.begin();
}

void MPU9250::init()
{
    uint8_t magData[3];

    // Wake up
    writeByte(I2C_ADDRESS, PWR_MGMT_1, 0x00);
    delay(100);

    // Get time source
    writeByte(I2C_ADDRESS, PWR_MGMT_1, 0x01);
    delay(200);

    // Set Gyro and Temp bandwidth to 41 Hz and 42 Hz
    // Limit sample rate to 1 kHz each
    writeByte(I2C_ADDRESS, CONFIG, 0x03);

    // Sample rate = Gyro output / (1 + SMPLRT_DIV)
    // 200 hz
    writeByte(I2C_ADDRESS, SMPLRT_DIV, 0x04);

    // Set Gyro full scale range
    uint8_t conf = readByte(I2C_ADDRESS, GYRO_CONFIG);
    writeByte(I2C_ADDRESS, GYRO_CONFIG, conf & ~0x02);
    writeByte(I2C_ADDRESS, GYRO_CONFIG, conf & ~0x18);
    writeByte(I2C_ADDRESS, GYRO_CONFIG, conf | GyroScale << 3);

    // Set Accel full scale range
    conf = readByte(I2C_ADDRESS, ACCEL_CONFIG);
    writeByte(I2C_ADDRESS, ACCEL_CONFIG, conf & ~0x18);//Nathan changed from ACCEL_CONFIG_2
    writeByte(I2C_ADDRESS, ACCEL_CONFIG, conf | AccelScale << 3);//same here

    // Set Accel sample rate
    conf = readByte(I2C_ADDRESS, ACCEL_CONFIG_2);
    writeByte(I2C_ADDRESS, ACCEL_CONFIG_2, conf & ~0x0F);
    writeByte(I2C_ADDRESS, ACCEL_CONFIG_2, conf | 0x03);

    // Configure interrupts and enable bypass mode
    writeByte(I2C_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(I2C_ADDRESS, INT_ENABLE, 0x01);
    delay(100);

    //----- Magnetometer -----

    //Power down magnetometer;
    // writeByte(MAG_ADDRESS, MAG_CNTL, 0x00);
    // delay(10);
    //
    // // Enter Fuse ROM access mode
    // writeByte(MAG_ADDRESS, MAG_CNTL, 0x0F);
    // delay(10);
    //
    // // Get calibration
    // readBytes(MAG_ADDRESS, MAG_ASAX, 3, &magData[0]);
    // magOffset[0] = (float)(magData[0] - 128) / 256.0 + 1.0;
    // magOffset[1] = (float)(magData[1] - 128) / 256.0 + 1.0;
    // magOffset[2] = (float)(magData[2] - 128) / 256.0 + 1.0;
    //
    // //Power down magnetometer;
    // writeByte(MAG_ADDRESS, MAG_CNTL, 0x00);
    // delay(10);

    // Configure the magnetometer for continuous read and highest resolution
    // writeByte(MAG_ADDRESS, MAG_CNTL, MagScale << 4 | magMode);
    // delay(10);

    setGyroRes();
    setAccelRes();
    //setMagRes();
}

void MPU9250::calibrate()
{
    uint8_t data[12];
    uint16_t index, cycleCount, fifoCount;
    int32_t accelFactory[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    uint32_t temperatureMask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t maskBit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    int32_t gyroOffset[3] = {0, 0, 0};

    //Reset
    writeByte(I2C_ADDRESS, PWR_MGMT_1, 0x80);
    delay(100);

    // Set time source
    writeByte(I2C_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(I2C_ADDRESS, PWR_MGMT_2, 0x00);
    delay(200);

    // Prepare device for offset calculation
    writeByte(I2C_ADDRESS, INT_ENABLE, 0x00);
    writeByte(I2C_ADDRESS, FIFO_EN, 0x00);
    writeByte(I2C_ADDRESS, PWR_MGMT_1, 0x00);
    writeByte(I2C_ADDRESS, I2C_MST_CTRL, 0x00);
    writeByte(I2C_ADDRESS, USER_CTRL, 0x00);
    writeByte(I2C_ADDRESS, USER_CTRL, 0x0C);
    delay(15);

    // Prepare gyro and accel for offset calculation
    writeByte(I2C_ADDRESS, CONFIG, 0x01);
    writeByte(I2C_ADDRESS, SMPLRT_DIV, 0x00);
    writeByte(I2C_ADDRESS, GYRO_CONFIG, 0x00);
    writeByte(I2C_ADDRESS, ACCEL_CONFIG, 0x00);

    // Configure FIFO for offset calculation
    writeByte(I2C_ADDRESS, USER_CTRL, 0x40);
    writeByte(I2C_ADDRESS, FIFO_EN, 0x78);
    delay(40);

    // Stop FIFO and read number of cycles
    writeByte(I2C_ADDRESS, FIFO_EN, 0x00);
    readBytes(I2C_ADDRESS, FIFO_COUNTH, 2, &data[0]);
    fifoCount = ((uint16_t)data[0] << 8) | data[1];
    cycleCount = fifoCount / 12;

    for (index = 0; index < cycleCount; index++)
    {
        int16_t gyroTemp[3] = {0, 0, 0}, accelTemp[3] = {0, 0, 0};
        readBytes(I2C_ADDRESS, FIFO_R_W, 12, &data[0]);
        accelTemp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]);
        accelTemp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]);
        accelTemp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]);
        gyroTemp[0] = (int16_t) (((int16_t)data[6] << 8) | data[7]);
        gyroTemp[1] = (int16_t) (((int16_t)data[8] << 8) | data[9]);
        gyroTemp[2] = (int16_t) (((int16_t)data[10] << 8) | data[11]);

        AccelOffset[0] += (int32_t) accelTemp[0];
        AccelOffset[1] += (int32_t) accelTemp[1];
        AccelOffset[2] += (int32_t) accelTemp[2];
        gyroOffset[0]  += (int32_t) gyroTemp[0];
        gyroOffset[1]  += (int32_t) gyroTemp[1];
        gyroOffset[2]  += (int32_t) gyroTemp[2];
    }

    AccelOffset[0] /= (int32_t) cycleCount;
    AccelOffset[1] /= (int32_t) cycleCount;
    AccelOffset[2] /= (int32_t) cycleCount;
    gyroOffset[0] /= (int32_t) cycleCount;
    gyroOffset[1] /= (int32_t) cycleCount;
    gyroOffset[2] /= (int32_t) cycleCount;

    // Make sure we don't remove gravity.
    if(AccelOffset[2] > 0L) {AccelOffset[2] -= (int32_t) AccelSensitivity;}
    else {AccelOffset[2] += (int32_t) AccelSensitivity;}

    data[0] = (-gyroOffset[0]/4 >> 8) & 0xFF;
    data[1] = (-gyroOffset[0]/4) & 0xFF;
    data[2] = (-gyroOffset[1]/4 >> 8) & 0xFF;
    data[3] = (-gyroOffset[1]/4) & 0xFF;
    data[4] = (-gyroOffset[2]/4 >> 8) & 0xFF;
    data[5] = (-gyroOffset[2]/4) & 0xFF;

    // Write gyro offsets.
    writeByte(I2C_ADDRESS, XG_OFFSET_H, data[0]);
    writeByte(I2C_ADDRESS, XG_OFFSET_L, data[1]);
    writeByte(I2C_ADDRESS, YG_OFFSET_H, data[2]);
    writeByte(I2C_ADDRESS, YG_OFFSET_L, data[3]);
    writeByte(I2C_ADDRESS, ZG_OFFSET_H, data[4]);
    writeByte(I2C_ADDRESS, ZG_OFFSET_L, data[5]);
}

uint8_t MPU9250::whoAmI()
{
    return readByte(I2C_ADDRESS, WHO_AM_I);
}

uint8_t MPU9250::whoAmIMag()
{
    return readByte(MAG_ADDRESS, MAG_WHO_AM_I);
}

void MPU9250::readAccelData(int16_t* dest)
{
  uint8_t rawData[6];
  readBytes(I2C_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
  dest[0] = (((int16_t)rawData[0] << 8) | rawData[1]) - AccelOffset[0];
  dest[1] = (((int16_t)rawData[2] << 8) | rawData[3]) - AccelOffset[1];
  dest[2] = (((int16_t)rawData[4] << 8) | rawData[5]) - AccelOffset[2];
}

void MPU9250::readGyroData(int16_t* dest)
{
  uint8_t rawData[6];
  readBytes(I2C_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
  dest[0] = (((int16_t)rawData[0] << 8) | rawData[1]);
  dest[1] = (((int16_t)rawData[2] << 8) | rawData[3]);
  dest[2] = (((int16_t)rawData[4] << 8) | rawData[5]);
}

void MPU9250::readMagData(int16_t* dest)
{
    uint8_t rawData[7];
    if (readByte(MAG_ADDRESS, MAG_ST1) & 0x01)
    {
        readBytes(MAG_ADDRESS, MAG_XOUT_L, 7, &rawData[0]);
        uint8_t c = rawData[6];
        if (!(c & 0x08))
        {
            dest[0] = (((int16_t)rawData[1] << 8) | rawData[0]);
            dest[1] = (((int16_t)rawData[3] << 8) | rawData[2]);
            dest[2] = (((int16_t)rawData[5] << 8) | rawData[4]);
        }
    }
}

void MPU9250::complementaryFilter(int16_t accel[3], int16_t gyro[3], float* pitch, float* roll, float* yaw)
{
    float gx, gy, gz;
    float pAccel, rAccel;

    gx = (float)gyro[0] * GyroRes;
    gy = (float)gyro[1] * GyroRes;
    gz = (float)gyro[2] * GyroRes;

    *pitch += gx * DeltaTime;
    *roll += gy * DeltaTime;
    *yaw += gz * DeltaTime;

    uint16_t force = abs(accel[0]) + abs(accel[1]) + abs(accel[2]);
    if (force > 8192 && force < 32768)
    {
        pAccel = atan2f(accel[1], accel[2]) * 180.0 / M_PI;
        *pitch = *pitch * 0.98 + pAccel * 0.02;

        rAccel = atan2f(accel[0], accel[2]) * 180.0 / M_PI;
        *roll = *roll * 0.98 + rAccel * 0.02;
    }
}

// ----- Private -----

// Res = Range / pow(2, Resolution-1)
// 16 bit resolution
// pow(2, 16-1) = 32768.0
void MPU9250::setGyroRes()
{
    switch (GyroScale)
    {
        case GFS_250DPS:
            GyroRes = 250.0 / 32768.0;
            break;
        case GFS_500DPS:
            GyroRes = 500.0 / 32768.0;
            break;
        case GFS_1000DPS:
            GyroRes = 1000.0 / 32768.0;
            break;
        case GFS_2000DPS:
            GyroRes = 2000.0 / 32768.0;
            break;
    }
}

// Res = Range / pow(2, Resolution-1)
// 16 bit resolution
// pow(2, 16-1) = 32768.0
void MPU9250::setAccelRes()
{
    switch (AccelScale)
    {
        case AFS_2G:
            AccelRes = 2.0 / 32768.0;
            break;
        case AFS_4G:
            AccelRes = 4.0 / 32768.0;
            break;
        case AFS_8G:
            AccelRes = 8.0 / 32768.0;
            break;
        case AFS_16G:
            AccelRes = 16.0 / 32768.0;
            break;
    }
}

// Res = Range / pow(2, Resolution-1)
// 16 bit resolution
// pow(2, 16-1) = 32768.0
// void MPU9250::setMagRes()
// {
//     switch (MagScale)
//     {
//         case MFS_14BIT:
//             MagRes = 10.0 * 4912.0 / 8190.0;
//             break;
//         case MFS_16BITS:
//             MagRes = 10.0 * 4912.0 / 32760.0;
//             break;
//     }
// }

void MPU9250::writeByte(uint8_t addr, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void MPU9250::writeBytes(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t length)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);

    for (int i = 0; i < length; i++)
    {
        Wire.write((uint8_t)data[i]);
    }

    Wire.endTransmission();
}

uint8_t MPU9250::readByte(uint8_t addr, uint8_t reg)
{
    uint8_t data;

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)addr, (uint8_t) 1);
    data = Wire.read();

    return data;
}

void MPU9250::readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* dest)
{
    uint8_t i = 0;

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)addr, (uint8_t)count);

    while (Wire.available())
    {
        dest[i++] = Wire.read();
    }
}
