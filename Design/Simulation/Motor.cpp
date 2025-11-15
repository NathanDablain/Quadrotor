#include "Motor.h"
#include "Gaussian.h"
#include "Low_Pass_Filter.h"

void Motor::Update_speed(){
    // Here the speed noise will be a percent of the current speed
    static Gaussian Speed_Noise(0.025, 0.0);
    if (!filter_initialized){
        speed_filter.Initialize(default_motor_bandwidth, 0.0);
        filter_initialized = true;
    }
    uint16_t Throt = (Throttle <= 1000)?Throttle:1000;
    if (Throt < deadzone){
        w = 0.0;
        return;
    }
    // Assume the motors ability to respond to control inputs varies with speed,
    // increasing as the speed increases. The bandwidth will be equal to (rpm/60)*12
    // as there are 12 coils in the motor with a FOS multipler of 0.75
    // Assume 0.02s rise time, and rise time = 0.35 / bandwidth -> bandwidth = 0.35/rise time
    // Equation for speed in rpm
    double speed;
    if (Throt <= 350){
        speed = static_cast<double>(Throt)*Motor_slope_l + Motor_zero_offset_l;
    }
    else{
        speed = static_cast<double>(Throt)*Motor_slope_h + Motor_zero_offset_h;
    }
    double w_input = speed*(1.0 + Speed_Noise.Get_val())*RPM2RADS;
    w = speed_filter.Update(w_input, 0.000333, false);
}

double Motor::Get_motor_thrust(){
    double thrust = k_f * pow(w, 2);
    return thrust;
}

double Motor::Get_motor_torque(){
    double torque = k_t * pow(w, 2);
    return torque;
}