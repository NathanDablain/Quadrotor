#ifndef IMU_H
#define	IMU_H

#include <stdint.h>

// Current IMU
#define LSM6DSR
// Legacy IMU
//#define LSM6DS3T

typedef enum{
    IMU_Standby,
    IMU_Fail,
    IMU_Ready,
    IMU_Reading
} IMU_Machine;

typedef struct{
    uint8_t accel_LSB_bytes[7];
    uint8_t gyro_LSB_bytes[7];
    int16_t gyro_LSB[3];
    int16_t accel_LSB[3];
    int32_t accel_bias_LSB[3];
    double angular_rate[3];
    double acceleration[3];
    uint8_t Accel_Read_Array[7];
    uint8_t Gyro_Read_Array[9];
    Time Last_Update_Accel;
    Time Last_Update_Gyro;
    bool drdy_Flag_Accel;
    bool drdy_Flag_Gyro;
} IMU_Data;

#ifdef LSM6DSR
    #define IMU_WHO_AM_I 0x0F
    #define IMU_ID 0x6B

    #define IMU_CTRL1_XL 0x10
    #define IMU_ACCEL_HIGH_RES (1<<1)
    #define IMU_2G_RANGE (0b00<<2)
    #define IMU_4G_RANGE (0b10<<2)
    #define IMU_8G_RANGE (0b11<<2)
    #define IMU_16G_RANGE (0b01<<2)
    #define IMU_ODR_52HZ (0b0011<<4)
    #define IMU_ODR_104HZ (0b0100<<4)
    #define IMU_ODR_208HZ (0b0101<<4)
    #define IMU_ODR_416HZ (0b0110<<4)
    #define IMU_ODR_833HZ (0b0111<<4)
    #define IMU_ODR_1660HZ (0b1000<<4)
    #define IMU_ODR_3330HZ (0b1001<<4)

    #define IMU_CTRL2_G 0x11
    #define IMU_125_RANGE (0b0010)
    #define IMU_250_RANGE (0b0000)
    #define IMU_500_RANGE (0b0100)
    #define IMU_1000_RANGE (0b1000)
    #define IMU_2000_RANGE (0b1100)
    #define IMU_4000_RANGE (0b0001)

    #define IMU_CTRL4_C 0x13
    #define IMU_GYRO_LPF_EN (1<<1)

    #define IMU_CTRL6_C 0x15
    #define IMU_ACCEL_HP_MODE (1<<4)
    #define IMU_GYRO_BW0 0b000
    #define IMU_GYRO_BW1 0b001
    #define IMU_GYRO_BW2 0b010
    #define IMU_GYRO_BW3 0b011

    #define IMU_CTRL8_XL 0x17
    #define IMU_ACCEL_BW_2 (0b000<<5)
    #define IMU_ACCEL_BW_4 (0b000<<5)
    #define IMU_ACCEL_BW_10 (0b001<<5)
    #define IMU_ACCEL_BW_20 (0b010<<5)
    #define IMU_ACCEL_BW_45 (0b011<<5)
    #define IMU_ACCEL_BW_100 (0b100<<5)
    #define IMU_ACCEL_BW_200 (0b101<<5)
    #define IMU_ACCEL_BW_400 (0b110<<5)
    #define IMU_ACCEL_BW_800 (0b111<<5)


    #define ACCEL_DATA_START 0x28
    #define GYRO_DATA_START 0x22
#else
    #define IMU_WHO_AM_I 0x0F
    #define IMU_RESET 0b00000001
    #define IMU_REBOOT 0b10000000
    #define IMU_ID 0x6A

    #define IMU_CTRL1_XL 0x10
    #define IMU_ACCEL_LPF_EN (1<<1)
    #define IMU_ACCEL_BW0 (1<<0)
    #define IMU_2G_RANGE (0b00<<2)
    #define IMU_4G_RANGE (0b10<<2)
    #define IMU_8G_RANGE (0b11<<2)
    #define IMU_16G_RANGE (0b01<<2)

    #define IMU_CTRL2_G 0x11
    #define IMU_245_RANGE (0b00<<2)
    #define IMU_500_RANGE (0b01<<2)
    #define IMU_1000_RANGE (0b10<<2)
    #define IMU_2000_RANGE (0b11<<2)

    #define IMU_ODR_52HZ 0b00110000
    #define IMU_ODR_104HZ 0b01000000
    #define IMU_ODR_208HZ 0b01010000
    #define IMU_ODR_416HZ 0b01100000
    #define IMU_ODR_833HZ 0b01110000
    #define IMU_ODR_1660HZ 0b10000000
    #define IMU_ODR_3330HZ 0b10010000

    #define IMU_CTRL3_C 0x12
    #define IMU_BDU (1<<6)
    #define IMU_AUTO_INC (1<<2)

    #define IMU_CTRL4_C 0x13
    #define IMU_GYR_LPF_EN (1<<1)

    #define IMU_CTRL6_C 0x15
    /*ODR: 800Hz       1660Hz       3300Hz
     BW0   245Hz       315Hz        343Hz   
     BW1   195Hz       224Hz        234Hz
     BW2   155Hz       168Hz        172Hz
     BW3   293Hz       505Hz        925Hz*/
    #define IMU_GYR_LPF_BW0 0
    #define IMU_GYR_LPF_BW1 1
    #define IMU_GYR_LPF_BW2 2
    #define IMU_GYR_LPF_BW3 3

    #define IMU_CTRL7_G 0x16
    #define IMU_G_HPM (1<<7)
    #define IMU_G_HPF (1<<6)

    #define IMU_CTRL8_XL 0x17
    /*
     BW0    ODR/50
     BW1    ODR/100
     BW2    ODR/9
     BW3    ODR/400*/
    #define IMU_ACC_LPF_BW0 (0b100<<5)
    #define IMU_ACC_LPF_BW1 (0b101<<5)
    #define IMU_ACC_LPF_BW2 (0b110<<5)
    #define IMU_ACC_LPF_BW3 (0b111<<5)

    #define ACCEL_DATA_START 0x28
    #define GYRO_DATA_START 0x20
#endif

#define GYRO_DRDY_bm (1<<1)
#define ACCEL_DRDY_bm (1<<0)
#define IMU_STATUS 0x1E

// Converts LSB to dps
#define GYRO_SENS (500.0/32768.0)
// Converts LSB to gs
#define ACCEL_SENS (2.0/32768.0)

void Initialize_IMU_Machine();

void Run_IMU_Machine();

IMU_Machine Initialize_IMU();

void Convert_Accel();

void Convert_Gyro();

double IMU_Acceleration(uint8_t index);

double IMU_Angular_Rate(uint8_t index);

#endif
