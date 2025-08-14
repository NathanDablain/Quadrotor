#pragma once

#include <cstdint>
#include <cmath>
#include <array>
#include "Sim_Time.h"
#include "Sim_Types.h"
#include "Linear_Algebra.h"
#include "Environment.h"
#include "Gaussian.h"

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
        void Initialize(uint16_t gyro_odr, uint16_t accel_odr, uint16_t accel_watermark);
        void Sample_Acc(Environment &env, Vec &quaternion, Sim_Time &sim_t);
        void Sample_Gyr(Environment &env, Vec3 &w, Sim_Time &sim_t);
        void Read_FIFO(std::array<int16_t, 3> *out);
};