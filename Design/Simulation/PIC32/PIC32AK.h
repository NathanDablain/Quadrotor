#pragma once

#include "IMU_p32.h"
#include "Time_p32.h"
#include "Environment.h"
#include "Sim_Time.h"
#include "Barometer.h"
#include "Magnetometer.h"
#include "IMU.h"
#include "Sim_Types.h"
#include "Guidance_p32.h"

extern "C" {
    // Wrappers to call C functions more representative of flight software
    bool Compare_And_Update_cpp(const Time Time_1, const Time Delta_t, Time* Time_2);
    Time Time_Difference_cpp(Time Time_1, Time Time_2);
    void Ground_Filter_Predict_cpp();
    void Ground_Filter_Update_cpp(double mag_field[3]);
    void Air_Filter_Predict_cpp();
    void Air_Filter_Update_cpp();
    void Initialize_Air_Filter_cpp();
    // Allocates memory for kalman filters and initializes values
    bool Initialize_IMU_Filters_cpp();
    // Returns ground or air filter state 
    double Filter_data_cpp(uint8_t index, uint8_t filter);
    // Returns ground or air filter covariance
    double Filter_covariance_cpp(uint8_t index1, uint8_t index2, uint8_t filter);
    // Call whenever ODR is changed on gyro, sets rms noise level for kalman filter
    void Set_ODR_cpp(double gyro_ODR, double accel_ODR);
    // Passes gyro data to save in IMU
    void Read_Gyro_cpp(Time Current_Time, int16_t angular_rate_LSB[3]);
    // Passes accelerometer data to save in IMU
    void Read_Accel_cpp(int16_t acceleration_LSB[3]);
    // Outputs IMU data
    double IMU_Angular_Rate_cpp(uint8_t index);
    double IMU_Acceleration_cpp(uint8_t index);
    // Passes magnetometer data to save in mag
    void Read_Mag_cpp(int16_t field_LSB[3]);
    double Magnetometer_Field_cpp(uint8_t index);
    void Initialize_Mag_cpp();
    // Passes barometer data to save in bar
    void Read_Bar_cpp(uint32_t pressure_LSB, uint8_t status);
    double Barometer_Altitude_cpp();
    double Barometer_Pressure_cpp();
    double Barometer_Altitude_Dot_cpp();
    void Initialize_Bar_cpp();
    void Altitude_Filter_Predict_cpp();
    void Altitude_Filter_Update_cpp();
    // Reads throttle commands out of PIC controller
    void Read_Throttles_cpp(uint16_t Throttles_cpp[4]);
    void Guidance_Machine_cpp(uint32_t current_time);
    void Initialize_Guidance_Machine_cpp();
    Guidance_State Get_Guidance_State_cpp();
    // Runs altitude feedforward/feedback controller
    void Initialize_Controllers_cpp();
    void Thrust_Control_cpp();
    // Runs moment lead/lag compensator and updates motor throttles for export
    void Moment_Control_cpp();
};

class PIC32AK{
    private:
        // Gyro 
        Time Gyro_Timelast = {0};
        Time Gyro_Update_Time = {0};
        const int32_t Gyro_Rate = 3330;
        const int32_t Gyro_Rate_us = (1000000/Gyro_Rate);
        // Accel
        Time Accel_Timelast = {0};
        Time Accel_Update_Time = {0};
        const int32_t Accel_Rate = 1660;
        const int32_t Accel_Rate_us = (1000000/Accel_Rate);
        // Mag
        Time Mag_Timelast = {0};
        Time Mag_Update_Time = {0};
        const int32_t Mag_Rate = 100;
        const int32_t Mag_Rate_us = (1000000/Mag_Rate);
        // Bar
        Time Bar_Timelast = {0};
        Time Bar_Update_Time = {0};
        const int32_t Bar_Rate = 200;
        const int32_t Bar_Rate_us = (1000000/Bar_Rate);
        // Controllers
        Time Thrust_Controller_Timelast = {0};
        Time Thrust_Controller_Update_Time = {0};
        const int32_t Thrust_Controller_Rate = 50;
        const int32_t Thrust_Controller_Rate_us = (1000000/Thrust_Controller_Rate);
        Time Moment_Controller_Timelast = {0};
        Time Moment_Controller_Update_Time = {0};
        const int32_t Moment_Controller_Rate = 3000;
        const int32_t Moment_Controller_Rate_us = (1000000/Moment_Controller_Rate);
        // Filters
        Time Air_Filter_Timelast = {0};
        Time Air_Filter_Update_Time = {0};
        const int32_t Air_Filter_Rate = 50;
        const int32_t Air_Filter_Rate_us = (1000000/Air_Filter_Rate);
        // Printing 
        bool print_flag = true;
        Time Print_Timelast = {0};
        Time Print_Update_Time = {0};
        const int32_t Print_Rate = 5;
        const int32_t Print_Rate_us = (1000000/Print_Rate);
        void Prep_Output();
        // General IMU
        bool IMU_Filter_Status;
        bool Air_Filter_Initialized;
    public:
        bool Successful_Landing = false;
        // LPS22H Barometer
        Barometer barometer;
        // LIS2MDL Magnetometer
        Magnetometer magnetometer;
        // LSM6DS3TR IMU
        IMU imu;
        // For the express purpose of logging
        States Output_States = {0};
        // Internal state machine
        FC_Status_p32 Flight_Controller_Status = Standby_p32;
        // Throttle commands 
        uint16_t mapped_throttle_commands[4] = {0};
        PIC32AK();
        void Run(Environment &env, Sim_Time sim_t);
};