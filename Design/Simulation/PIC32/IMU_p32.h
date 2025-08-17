#ifndef IMU_P32_H
#define IMU_P32_H

#include "Kalman_Filter_p32.h"
#include <stdint.h>

typedef struct {
    // Kalman filters for the gyroscopes aligned with the drone body x-y-z axes
    Kalman_Filter *Gyro_Filter_x;
    Kalman_Filter *Gyro_Filter_y;
    Kalman_Filter *Gyro_Filter_z;
    // Raw output from the gyroscopes
    int16_t gyro_output_LSB[3];
    // True angular rate estimate from kalman filters, in deg/s
    float w_filtered[3];
} IMU_Data;

uint8_t Filter_Gyro(Kalman_Filter *Gyro_Filter, int16_t gyro_output_LSB);

#endif