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

#ifndef _MPU9250_H_
#define _MPU9250_H_

#include<Wire.h>
#include<Arduino.h>

// Gyroscope Self-Test Registers
#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0x01
#define SELF_TEST_Z_GYRO    0x02

// Accelerometer Self-Test Registers
#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0x0F

// Gyro Offset Registers
#define XG_OFFSET_H         0x13
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18

// Sample Rate Divider
#define SMPLRT_DIV          0x19

// Configuration
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG_2      0x1D
#define LP_ACCEL_ODR        0x1E

// Wake-on Motion Threshold
#define WOM_THR             0x1F

// FIFO Enable
#define FIFO_EN             0x23

// I2C Master Control
#define I2C_MST_CTRL        0x24

// I2C Slave Control
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36

// Interrupts
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define INT_STATUS          0x3A

// Accelerometer Measurements
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

// Temperature Measurements
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42

// Gyroscope Measurements
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

// External Sensor Data
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60

// I2C Slave Data Out
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66

// I2C Master Delay Control
#define I2C_MST_DELAY_CTRL  0x67

// Signal Path Reset
#define SIGNAL_PATH_RESET   0x68

// Accelerometer Interrupt Control
#define MOT_DETECT_CTRL     0x69

// User Control
#define USER_CTRL           0x6A

// Power Management
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C

// FIFO Count Registers
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73

// FIFO Read/Write
#define FIFO_R_W            0x74

// Who Am I - Should = 0x71
#define WHO_AM_I            0x75

// Accelerometer Offset Registers
#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E

// Magnetometer Who Am I - Should = 0x48
#define MAG_WHO_AM_I        0x00

// Magnetometer Info
#define MAG_INFO            0x01

// Magnetometer Status
#define MAG_ST1             0x02
#define MAG_ST2             0x09

// Magnetometer Measurements
#define MAG_XOUT_L          0x03
#define MAG_XOUT_H          0x04
#define MAG_YOUT_L          0x05
#define MAG_YOUT_H          0x06
#define MAG_ZOUT_L          0x07
#define MAG_ZOUT_H          0x08

// Magnetometer Control
#define MAG_CNTL            0x0A

// Magnetometer Reserved
#define MAG_RSV             0x0B

// Magnetometer Tests
#define MAG_ADDRESS         0x0C
#define MAG_TS1             0x0D
#define MAG_TS2             0x0E

// Magnetometer I2C Disable
#define MAG_I2CDIS          0x0F

// Fuse ROM Sensitivity Adjustment
#define MAG_ASAX            0x10
#define MAG_ASAY            0x11
#define MAG_ASAZ            0x12

// Hardcoded for Now
#define I2C_ADDRESS         0x68

enum GyroScale
{
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum AccelScale
{
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum MagScale
{
    MFS_14BIT = 0,
    MFS_16BITS
};

class MPU9250
{
public:
    uint8_t GyroScale = GFS_250DPS;
    uint8_t AccelScale = AFS_4G;
    uint8_t MagScale = MFS_16BITS;

    uint16_t GyroSensitivity = 131;
    uint16_t AccelSensitivity = 16384;
    int32_t AccelOffset[3] = {0, 0, 0};
    //float MagOffset[3] = {0, 0, 0};
    //uint8_t MagMode = 0x02;
    float AccelRes, GyroRes;//, MagRes;
    float DeltaTime, Now, LastUpdate;

    MPU9250();
    void init();
    void calibrate();

    uint8_t whoAmI();
    uint8_t whoAmIMag();

    void readAccelData(int16_t* dest);
    void readGyroData(int16_t* dest);
    void readMagData(int16_t* dest);

    void complementaryFilter(int16_t accel[3], int16_t gyro[3], float* pitch, float* roll, float* yaw);
private:
    void setGyroRes();
    void setAccelRes();
    //void setMagRes();
    void writeByte(uint8_t addr, uint8_t reg, uint8_t data);
    void writeBytes(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t length);
    uint8_t readByte(uint8_t addr, uint8_t reg);
    void readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* dest);
};

#endif // _MPU9250_H_
