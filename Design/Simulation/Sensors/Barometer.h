#pragma once

#include <cstdint>
#include "Environment.h"
#include "Sim_Time.h"
#include "Gaussian.h"
#include "Low_Pass_Filter.h"

typedef enum {
    Bar_Mode_Bypass,
    Bar_Mode_FIFO
} Barometer_mode;

class Barometer{
    private:
        //------LPS22HH Barometer Parameters-----//
        // sensitivity in LSB/hpa
        const double sensitivity = 4096.0;
        // noise is in hpa, data sheet value plus 50% FOS
        const double noise_rms = 1.5*0.0170;
        // Output Data Rate in Hz
        uint16_t ODR;
        // Time between sensor readings in microseconds
        Sim_Time Update_rate = {0};
        Sim_Time last_sample_time;
        // Number of sensor readings at which FIFO is full
        uint8_t FIFO_watermark;
        // Determines if FIFO is used or readings are bypassed to output
        Barometer_mode Mode;
        // FIFO buffer
        uint32_t FIFO_buffer[255];
        // Low pass filter setting -> BW = ODR/Low_Pass_Filter_Setting
        double Low_Pass_Filter_BW[3] = {2.0, 9.0, 20.0};
        uint8_t Filter_Setting;
        bool passthrough_flag = false;
        Gaussian Gaussian_Bar;
        Low_Pass_Filter Bar_Filter;
    public:
        bool drdy_flag = false;
        // The data in the readable output registers
        uint32_t Pressure_Out_LSB;
        // FIFO index
        uint32_t FIFO_index = 0;
        // Set data rate, and enable FIFO
        void Initialize(uint16_t odr, uint8_t watermark, Barometer_mode mode, uint8_t filter_setting);
        // Turns true pressure data tracked in environment into a quantized LSB reading
        void Sample(Environment &env, Sim_Time sim_t);
        // Fills supplied array with data in FIFO
        void Read_FIFO(uint32_t *out);
};