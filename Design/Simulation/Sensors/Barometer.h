#pragma once

#include <cstdint>
#include "Environment.h"
#include "Sim_Time.h"
#include "Gaussian.h"
#include "Low_Pass_Filter.h"

class Barometer{
    private:
        //------LPS22HH Barometer Parameters-----//
        // sensitivity in LSB/hpa
        const double sensitivity = 4096.0;
        // noise is in hpa, first with low pass en = true, ODR/2, ODR/9, ODR/20 
        double noise_rms[2][3] = {{0.017, 0.009, 0.0065}, {0.045, 0.026, 0.017}};
        // Output Data Rate in Hz
        uint16_t ODR;
        // Time between sensor readings in microseconds
        Sim_Time Update_rate = {0};
        Sim_Time last_sample_time;
        // Low pass filter setting -> BW = ODR/Low_Pass_Filter_Setting, ODR/2 is the default
        double Low_Pass_Filter_BW[3] = {2.0, 9.0, 20.0};
        bool low_noise_en = false;
        const bool passthrough_flag = false;
        Gaussian Gaussian_Bar;
        Low_Pass_Filter Bar_Filter;
    public:
        bool drdy_flag = false;
        // The data in the readable output registers
        uint32_t Pressure_Out_LSB;
        // FIFO index
        uint32_t FIFO_index = 0;
        // Set data rate, and enable FIFO
        void Initialize(uint16_t odr, bool low_noise, uint8_t filter_setting);
        // Turns true pressure data tracked in environment into a quantized LSB reading
        void Sample(Environment &env, Sim_Time sim_t);
        // Fills supplied array with data in FIFO
        void Read_FIFO(uint32_t *out);
};