#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include "Sim_Types.h"

using namespace std;

#define RPM2RADS (PI/30.0) 
#define RADS2RPM (1.0/RPM2RADS)

class Motor{
    private:
        // Current motor speed in rad/s
        double w;
        // Throttle command from ESC: 0 - 1000
        double throttle;
        // Propeller thrust constant in N/(rad/s)^2 
        double k_f = 0.000001;
        // Propeller torque constant in N-m/(rad/s)^2
        double k_t = 0.000000011;
        // Motor speed constant in Rad/s/V
        double KV = 1500.0*RPM2RADS;
        // Motor torque constant in N-m/A (1/KV)
        double KT = 1.0/KV;
        // This array holds the current produced by the motor for each 10% of throttle, starting at 0%
        array<double, 11> Current = {0.0, 0.1, 0.7, 2.0, 4.1, 7.2, 10.9, 15.4, 20.5, 25.9, 31.8};

    public:
        // Motor specific, found emperically, in throttle units of 0-1000
        uint16_t deadzone = 0; 
        // Throttle command will range from 3000-6000, corresponding to 0-1000 throttle
        uint16_t Throttle = 3000;
        void
            Update_speed();
        double  
            Get_motor_thrust(),
            Get_motor_torque();
};