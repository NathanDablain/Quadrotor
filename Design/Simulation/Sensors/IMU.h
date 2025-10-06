#pragma once

#include <cstdint>
#include <cmath>
#include <array>
#include "Sim_Time.h"
#include "Sim_Types.h"
#include "Linear_Algebra.h"
#include "Environment.h"
#include "Gaussian.h"
#include "Low_Pass_Filter.h"

class IMU{
    private:
        // Sensor Output Data Rates in us, used to track updates
        Sim_Time Update_Rate_Gyro;
        Sim_Time Update_Rate_Accel;
        Sim_Time last_sample_time_gyro;
        Sim_Time last_sample_time_accel;
        // Sensor Output Data Rates in (Hz), will drive noise variance
        uint16_t ODR_Gyro;
        uint16_t ODR_Accel;
        //----LSM6DS3TR Gyroscope Parameters-----//
        // Range of gyro measurements in +-dps
        const double gyro_range = 500.0;
        // Rate noise RMS is defined as 5 mdps/sqrt(Hz), 
        double gyro_noise_rms;
        // Sensitivity is in units of mdps/LSB
        const double gyro_sens_mdps = 17.5;
        const double gyro_sens_dps = gyro_sens_mdps/1000.0;
        // Gyro random walk is assumed to be 5 dph -> 0.001388 dps -> 1.388 mdps
        const double gyro_random_walk_rate_mdps = 1.388;
        double gyro_random_walk_mdps;
        Sim_Time time_last_walk_update;
        Sim_Time Random_walk_rate;
        // Bias is a constant offset applied in mdps, range is +-3 dps
        Vec3 gyro_bias;
        // There are four low pass filter bandwidths available for gyro ODRs above 833 Hz
        double Gyro_ODR_833_BW[4] = {245.0, 195.0, 155.0, 293.0};
        double Gyro_ODR_1660_BW[4] = {315.0, 224.0, 168.0, 505.0};
        double Gyro_ODR_3330_BW[4] = {343.0, 234.0, 172.0, 925.0};
        double Gyro_ODR_6660_BW[4] = {351.0, 237.0, 173.0, 937.0};
        uint8_t Gyro_filter_setting;
        bool Gyro_passthrough_flag = false;
        double Gyro_Filter_BW;
        Gaussian gyro_gaussian;
        Low_Pass_Filter gyro_filter_x;
        Low_Pass_Filter gyro_filter_y;
        Low_Pass_Filter gyro_filter_z;
        //---------------------------------------//
        //--LSM6DS3TR Accelerometer Parameters---//
        // Range of accel measurements in +-mg
        const double accel_range = 2.0*1000.0;
        // Accel noise RMS is defined as 90ug/sqrt(Hz) -> 0.09 mg/sqrt(Hz)
        double accel_noise_rms;
        // Sensitivity is in units of mg/LSB
        const double accel_sens = 0.061;
        // Bias is a constant offset applied in mg, range is +-40 mg
        Vec3 accel_bias;
        // Bandwidth modifier of low pass filter -> BW = ODR_Accel/Accel_filter_BW
        double Accel_filter_BW[4] = {9.0, 50.0, 100.0, 400.0};
        uint8_t Accel_filter_setting;
        bool Accel_passthrough_flag = false;
        Gaussian accel_gaussian;
        Low_Pass_Filter accel_filter_x;
        Low_Pass_Filter accel_filter_y;
        Low_Pass_Filter accel_filter_z;
        //---------------------------------------//
        std::array<int16_t, 3> FIFO_buffer[255];
        uint16_t FIFO_watermark;
    public:
        uint8_t FIFO_index;
        bool gyro_drdy_flag = false;
        bool accel_drdy_flag = false;
        // The below are SENSED variables, they are read by the flight controllers sensors
        std::array<int16_t, 3> angular_rate_LSB;
        std::array<int16_t, 3> acceleration_LSB;
        void Initialize(uint16_t gyro_odr, uint16_t accel_odr, uint16_t accel_watermark, uint8_t gyro_lpf_setting, uint8_t accel_lpf_setting);
        void Sample_Acc(Environment &env, Sim_Time &sim_t);
        void Sample_Gyr(Environment &env, Vec3 &w, Sim_Time &sim_t);
        void Read_FIFO(std::array<int16_t, 3> *out);
        void Set_Filter_Settings(uint8_t gyro_lpf_setting, uint8_t accel_lpf_setting);
};