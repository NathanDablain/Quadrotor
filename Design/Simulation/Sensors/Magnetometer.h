#pragma once

#include <cstdint>
#include "Environment.h"
#include "Gaussian.h"
#include "Low_Pass_Filter.h"

class Magnetometer {
    private:
        Sim_Time Update_Rate;
        Sim_Time last_sample_time;
        uint16_t ODR;
        uint8_t filter_setting;
        //----LIS2MDL Magnetometer Parameters----//
        // Sensitivity is in units of mgauss/LSB
        const double mag_sens = 1.5;
        // RMS magnetometer noise in (mgauss),  data sheet value plus 50% FOS
        const double noise_rms = 1.5*3.0;
        // Low pass filter setting -> BW = ODR/Low_Pass_Filter_Setting
        double Low_Pass_Filter_BW[2] = {2.0, 4.0};
        bool passthrough_flag = false;
        Gaussian Gaussian_mag;
        Low_Pass_Filter Mag_Filter_x;
        Low_Pass_Filter Mag_Filter_y;
        Low_Pass_Filter Mag_Filter_z;
    public:
        // Output data
        array<int16_t, 3> magnetic_field_LSB;
        // Data ready flag, set when new data has been sampled
        bool drdy_flag = false;
        void Initialize(uint16_t odr, uint8_t lpf_setting);
        void Sample(Environment &env, Sim_Time sim_t);
};