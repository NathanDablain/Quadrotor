#pragma once

#include <cstdint>
#include <cmath>
#include <array>
#include "Sim_Time.h"
#include "Sim_Types.h"
#include "Linear_Algebra.h"
#include "Environment.h"

class IMU {
    private:
        // Sensor Output Data Rates in us, used to track updates
        Sim_Time Update_Rate_Gyro;
        Sim_Time Update_Rate_Accel;
        Sim_Time last_sample_time_gyro;
        Sim_Time last_sample_time_accel;
        // Sensor Output Data Rates in (Hz), will drive noise density
        uint16_t ODR_Gyro;
        uint16_t ODR_Accel;
        //----LSM6DS3TR Gyroscope Parameters-----//
        // Range of gyro measurements in +-rad/s
        const double gyro_range = 500.0*D2R;
        // Rate noise density is defined as 5/sqrt(Hz), here is in rad/s
        double gyro_max_noise;
        // Sensitivity is in units of rad/s/LSB
        const double gyro_sens = (17.5/1000.0)*D2R;
        //---------------------------------------//
        //--LSM6DS3TR Accelerometer Parameters---//
        // Range of accel measurements in +-g
        const double accel_range = 2.0;
        // Accel noise density is defined as 90/sqrt(Hz), here is in g
        double accel_max_noise;
        // Sensitivity is in units of g/LSB
        const double accel_sens = 0.000061;
        //---------------------------------------//
        std::array<int16_t, 3> FIFO_buffer[255];
        uint16_t FIFO_watermark;
    public:
        uint8_t FIFO_index;
        bool gyro_drdy_flag = false;
        // The below are SENSED variables, they are read by the flight controllers sensors
        std::array<int16_t, 3> angular_rate_LSB;
        std::array<int16_t, 3> acceleration_LSB;
        void Initialize(uint16_t gyro_odr, uint16_t accel_odr, uint16_t accel_watermark);
        void Sample_Acc(Environment &env, Vec &quaternion, Sim_Time &sim_t);
        void Sample_Gyr(Environment &env, Vec3 &w, Sim_Time &sim_t);
        void Read_FIFO(std::array<int16_t, 3> *out);
};