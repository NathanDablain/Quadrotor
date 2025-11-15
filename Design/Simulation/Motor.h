#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include "Sim_Types.h"
#include "Low_Pass_Filter.h"

using namespace std;

#define RPM2RADS (PI/30.0) 
#define RADS2RPM (1.0/RPM2RADS)

class Motor{
    private:
        // Current motor speed in rad/s
        double w = 0.0;
        // Throttle command from ESC: 0 - 1000
        double throttle = 0.0;
        double default_motor_bandwidth = 0.35/0.02;
        Low_Pass_Filter speed_filter;
        bool filter_initialized = false;
    public:
        // Propeller thrust constant in N/(rad/s)^2 
        double k_f;
        // Propeller torque constant in N-m/(rad/s)^2
        double k_t;
        // Constants that define linear fit of propeller speed to motor throttle
        double Motor_slope_l;
        double Motor_zero_offset_l;
        double Motor_slope_h;
        double Motor_zero_offset_h;
        // Motor specific, found emperically, in throttle units of 0-1000
        uint16_t deadzone = 0; 
        // Throttle command will range from 0-100
        uint16_t Throttle = 0;
        void
            Update_speed();
        double  
            Get_motor_thrust(),
            Get_motor_torque();
};