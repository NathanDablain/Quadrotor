#ifndef MAGNETOMETER_H
#define	MAGNETOMETER_H

#include <stdint.h>
#include "linear_algebra.h"

typedef enum{
    Mag_Standby,
    Mag_Fail,
    Mag_Ready,
    Mag_Reading
} Mag_Machine;

typedef struct{
    double field[3];
    uint8_t field_LSB_bytes[7];
    int16_t field_LSB[3];
    int16_t hard_iron[3];
    int16_t mag_values_at_max[3][3];
    int16_t mag_field_max_LSB[3];
    int16_t mag_field_min_LSB[3];
    Matrix_3 soft_iron;
} Mag_Data;

#define MAG_WHO_AM_I 0x4F
#define MAG_ID (1<<6)

#define MAG_CFG_REG_A 0x60
#define MAG_TEMP_COMP (1<<7)
#define MAG_REBOOT (1<<6)
#define MAG_SOFT_RESET (1<<5)
#define MAG_ODR_10Hz ((0<<3)|(1<<2))
#define MAG_ODR_20Hz ((0<<3)|(1<<2))
#define MAG_ODR_50Hz ((1<<3)|(0<<2))
#define MAG_ODR_100Hz ((1<<3)|(1<<2))

#define MAG_CFG_REG_B 0x61
// When disabled, the bandwidth is ODR/2
#define MAG_LPF_DISABLE (0<<0)
// When enabled, the bandwidth is ODR/4
#define MAG_LPF_ENABLE (1<<0)

#define MAG_CFG_REG_C 0x62
#define MAG_DISABLE_I2C (1<<5)
#define MAG_BDU (1<<4)
#define MAG_4WSPI (1<<2)

#define MAG_DATA_START 0x68
#define MAG_STATUS 0x67
#define MAG_DRDY_bm (1<<3)

#define MAG_SENSITIVITY 1.5

void Run_Magnetometer_Machine();

Mag_Machine Initialize_Magnetometer();

void Convert_Magnetometer();

bool Calculate_Hard_Iron();

bool Calculate_Soft_Iron();

void Compensate_Magnetometer_Reading(bool Soft_iron_cal);

double Magnetometer_Field(uint8_t index);

#endif

