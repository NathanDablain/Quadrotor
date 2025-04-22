#ifndef IMU_H
#define IMU_H

#include "FC_Types.h"

#define IMU_WHO_AM_I 0x0F
#define IMU_CTRL1_XL 0x10
#define IMU_CTRL2_G 0x11
#define IMU_CTRL8_XL 0x17
#define ACCEL_DATA_START 0x28
#define GYRO_DATA_START 0x22
#define IMU_WINDOW_SIZE 8 // Size of FIR window for accelerometer
#define GYRO_WINDOW_SIZE 4 // Size of FIR window for gyroscope
#define GYRO_SENS 500.0/32768.0
#define ACCEL_SENS 2.0/32768.0

#define W_CAL_LIMIT 500

unsigned char Setup_IMU();

void Read_Gyro(States *Drone, Calibration_Data *cal_data);

void Read_Accel(States *Drone);

void Calibrate_IMU(States *Drone, Calibration_Data *cal_data);

#endif