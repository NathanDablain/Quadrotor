#pragma once

#include <cstdint>
#include <cmath>

#define RPM2RADS M_PI/30.0 
#define RADS2RPM 1.0/RPM2RADS

class Motor{
    private:
        double
            // Current motor speed in rad/s
            w,
            // Throttle command from ESC: 0 - 100
            throttle,
            // Propeller thrust constant in N/(rad/s)^2 
            k_f = 0.000001,
            // Propeller torque constant in N-m/(rad/s)^2
            k_t = 0.000000011,
            // Motor speed constant in RPM/V
            KV,
            // Motor torque constant in N-m/A (1/KV)
            KT;
    public:
        void
            Send_throttle_command(double command);
        double  
            Get_motor_thrust(),
            Get_motor_torque();
};