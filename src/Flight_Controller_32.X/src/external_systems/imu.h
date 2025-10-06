#ifndef IMU_H
#define	IMU_H

#include <stdint.h>

typedef enum{
    IMU_Standby,
    IMU_Fail,
    IMU_Ready,
    IMU_Reading
} IMU_Machine;

typedef struct{
    uint8_t accel_LSB_bytes[7];
    uint8_t gyro_LSB_bytes[9];
    int16_t gyro_LSB[3];
    int16_t accel_LSB[3];
    int32_t accel_bias_LSB[3];
    double angular_rate[3];
    double acceleration[3];
} IMU_Data;

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

#define GYRO_DRDY_bm (1<<1)
#define ACCEL_DRDY_bm (1<<0)
#define IMU_STATUS 0x1E

// Converts LSB to dps
#define GYRO_SENS (500.0/32768.0)
// Converts LSB to gs
#define ACCEL_SENS (2.0/32768.0)

void Run_IMU_Machine();

IMU_Machine Initialize_IMU();

void Convert_Accel();

void Convert_Gyro();

double IMU_Acceleration(uint8_t index);

double IMU_Angular_Rate(uint8_t index);

#endif
