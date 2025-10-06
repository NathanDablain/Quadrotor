#ifndef IMU_P32_H
#define IMU_P32_H

#include "Kalman_Filter_p32.h"
#include "Time_p32.h"
#include "Sim_Types.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // Raw gyro output converted to dps
    double angular_rate[3];
    // Raw output from the gyroscopes
    int16_t gyro_output_LSB[3];
    // True angular rate estimate from kalman filters, in deg/s
    float w_filtered[3];
    // Raw output from the accelerometers
    int16_t accel_output_LSB[3];
    // Bias in accelerometer axis
    int32_t accel_bias_LSB[3];
    // Raw accelerometer output converted to gs
    double acceleration[3];
	int16_t accel_max[3];
    int16_t accel_min[3];
} IMU_Data;

typedef struct{
    // Diagonal terms of covariance matrix are variances
    double P1_1;
    double P2_2;
    double P3_3;
    double P4_4;
    double P5_5;
    double P6_6;
    // Off diagonal terms are covariances
    double P1_2;
    double P1_3;
    double P1_4;
    double P1_5;
    double P1_6;

    double P2_3;
    double P2_4;
    double P2_5;
    double P2_6;

    double P3_4;
    double P3_5;
    double P3_6;

    double P4_5;
    double P4_6;

    double P5_6;
} Air_Filter_Covariance;

void Ground_Filter_Predict();

void Ground_Filter_Update(double mag_field[3]);

void Ground_Filter_State_Transition();

void Air_Filter_Predict();

void Air_Filter_Predict_p32();

void Air_Filter_Update();

void Air_Filter_Update_p32();

void Air_Filter_State_Transition();

bool Initialize_IMU_Filters();

void Initialize_Air_Filter();

double Filter_data(uint8_t index, uint8_t filter);

double Filter_covariance(uint8_t index1, uint8_t index2, uint8_t filter);

void Set_ODR(double gyro_ODR, double accel_ODR);

void Read_Accel(int16_t acceleration_LSB[3]);

double IMU_Acceleration(uint8_t index);

void Read_Gyro(Time Current_Time, int16_t angular_rate_LSB[3]);

double IMU_Angular_Rate(uint8_t index);

double Air_Filter_x_Dot(uint8_t index);

void Altitude_Filter_Predict();

void Altitude_Filter_Update();

void Altitude_Filter_Predict_p32();

void Altitude_Filter_Update_p32();

#endif