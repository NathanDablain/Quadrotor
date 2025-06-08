#pragma once

#include <cstdint>
#include "Environment.h"

class Magnetometer {
    private:
    Sim_Time Update_Rate;
    Sim_Time last_sample_time;
    uint16_t ODR;
    //----LIS2MDL Magnetometer Parameters----//
    // Sensitivity is in units of mgauss/LSB
    const double mag_sens = 1.5;
    // RMS magnetometer noise in (mgauss)
    const double mag_max_noise = 30.0; //3;
    const double mag_noise_sens = mag_max_noise/32767.0;
    public:
        // Output data
        array<int16_t, 3> magnetic_field_LSB;
        // Data ready flag, set when new data has been sampled
        bool drdy_flag;
        void Initialize(uint16_t odr);
        void Sample(Environment &env, Sim_Time sim_t);
};