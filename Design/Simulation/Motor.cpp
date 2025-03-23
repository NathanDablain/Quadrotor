#include "Motor.h"

void Motor::Send_throttle_command(double command){

}

double Motor::Get_motor_thrust(){
    double thrust = k_f * pow(w, 2);
    return thrust;
}

double Motor::Get_motor_torque(){
    double torque = k_t * pow(w, 2);
    return torque;
}