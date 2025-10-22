#ifndef IMU_P32_H
#define IMU_P32_H

#include "Sim_Types.h"
#include "Time_p32.h"
#include <stdint.h>

typedef enum{
    IMU_Standby,
    IMU_Fail,
    IMU_Ready,
    IMU_Reading
} IMU_Machine;

typedef struct {
    // Raw gyro output converted to dps
    double angular_rate[3];
    // Raw output from the gyroscopes
    uint8_t gyro_LSB_bytes[6];
    int16_t gyro_LSB[3];
    // True angular rate estimate from kalman filters, in deg/s
    float w_filtered[3];
    // Raw output from the accelerometers
    uint8_t accel_LSB_bytes[6];
    int16_t accel_LSB[3];
    // Bias in accelerometer axis
    int32_t accel_bias_LSB[3];
    // Raw accelerometer output converted to gs
    double acceleration[3];
	int16_t accel_max[3];
    int16_t accel_min[3];
    Time Last_Update_Accel;
    Time Last_Update_Gyro;
} IMU_Data;

void Initialize_IMU();

void Run_IMU_Machine();

void Convert_Accel();

double IMU_Acceleration(uint8_t index);

void Convert_Gyro();

double IMU_Angular_Rate(uint8_t index);

#endif