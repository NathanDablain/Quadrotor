#include "Motor.h"
#include "Gaussian.h"

void Motor::Update_speed(){
    // Here the speed noise will be a percent of the current speed
    static Gaussian Speed_Noise(0.025, 0.0);
    uint16_t Throt = (Throttle <= 1000)?Throttle:1000;
    if (Throt < deadzone){
        w = 0.0;
        return;
    }
    // Equation for speed in rpm
    double speed = (static_cast<double>(Throt)/10.0)*Motor_slope + Motor_zero_offset;
    w = speed*(1.0 + Speed_Noise.Get_val())*RPM2RADS;
}

double Motor::Get_motor_thrust(){
    double thrust = k_f * pow(w, 2);
    return thrust;
}

double Motor::Get_motor_torque(){
    double torque = k_t * pow(w, 2);
    return torque;
}