#include "Motor.h"

void Motor::Update_speed(){
    // convert throttle to integer between 0-1000
    // divide by 100 to get  
    // interp to find current based on throttle
    uint16_t Throt = (Throttle-3000)/3;
    if (Throt < deadzone){
        w = 0.0;
        return;
    }
    double to_interp = static_cast<double>(Throt)/100.0;
    uint8_t lower_index = static_cast<uint8_t>(floor(to_interp));
    uint8_t upper_index = static_cast<uint8_t>(ceil(to_interp));
    if (upper_index > (Current.size() - 1)){
        upper_index -= 1;
    }
    else if (lower_index < 0){
        lower_index += 1;
    }
    double current_interp = (Current[upper_index] - Current[lower_index])*(to_interp - static_cast<double>(lower_index)) + Current[lower_index];
    // Using motor torque constant and propeller torque constant, convert current to speed
    double speed = sqrt(KT*current_interp/k_t);
    w = w*0.95 + speed*0.05;
}

double Motor::Get_motor_thrust(){
    double thrust = k_f * pow(w, 2);
    return thrust;
}

double Motor::Get_motor_torque(){
    double torque = k_t * pow(w, 2);
    return torque;
}