#include "Controllers.h"

float Height_LQR(float h, float h_ref){
    // u = -K*x -> x = h_int, h, h_dot
    const float mass = 0.45;
    const float d_t = 0.01;
    float K[2] = {3.0, 4.0};
    float K_int = 0.002;
    static float e_int;
    static float h_last;
    e_int += (h_ref-h);
    float h_dot = (h-h_last)/d_t;
    h_last = h;
    float u = -K[0]*(h-h_ref) - K[1]*h_dot + K_int*e_int + 9.81;
    float thrust = mass*u;
    return thrust; 
}

void Angular_Rate_Control(States &MCU, States &Reference, float desired_moments[3]){
    // const float I[3] = {0.00149, 0.00262, 0.00149};
    const float d_t = 0.0025;
    const float c1 = 0.9;
    const float c2 = 1.0-c1;
    const float K_i = 0.001;
    float K[3][2] = {{123.7, -1483.8},{223.9,-2687.2},{27.1, -3250.9}};
    static float Euler_last[3];
    static float Euler_dot[3];
    static float e_int[3];
    for (uint8_t i = 0; i < 3; i++){
        float e = Reference.Euler[i]-MCU.Euler[i];
        e_int[i] += e;
        Euler_dot[i] = Euler_dot[i]*c1 + ((MCU.Euler[i]-Euler_last[i])/d_t)*c2;
        Euler_last[i] = MCU.Euler[i];
        desired_moments[i] = K[i][0]*e - K[i][1]*Euler_dot[i] - K_i*e_int[i];
    }
    
}

void Set_throttles(uint16_t motor_throttles[4], float desired_base_speed, float desired_delta[3]){
    // -> Back motor (0) produces negative pitching torque and negative yawing torque
    // -> Left motor (1) produces positive rolling torque and positive yawing torque
    // -> Right motor (2) produces negative rolling torque and positive yawing torque
    // -> Front motor (3) produces positive pitching torque and negative yawing torque
    // Propeller thrust constant in N/(rad/s)^2 
    const float k_f = 0.000001;
    // Propeller torque constant in N-m/(rad/s)^2
    const float k_t = 0.000000011;
    // Distance from front and back motor thrust vectors to drone center of gravity in (m)
    const float length_f_b =  0.117;
    // Distance from left and right motor thrust vectors to drone center of gravity in (m)
    const float length_l_r = 0.1205;
    const float denom_1 = 4.0*k_f*k_t*length_f_b;
    const float denom_2 = 4.0*k_f*k_t*length_l_r;
    const float c1 = k_f*length_f_b;
    const float c2 = k_t*length_f_b;
    const float c3 = 2.0*k_t;
    const float c4 = k_f*length_l_r;
    const float c5 = k_t*length_l_r;

    // w_f: (2*My*kt - Mz*kf*lfb + T*kt*lfb)/(4*kf*kt*lfb)
    // w_r: (Mz*kf*lrl - 2*Mx*kt + T*kt*lrl)/(4*kf*kt*lrl)
    // w_l: (2*Mx*kt + Mz*kf*lrl + T*kt*lrl)/(4*kf*kt*lrl)
    // w_b: -(2*My*kt + Mz*kf*lfb - T*kt*lfb)/(4*kf*kt*lfb)

    // float omega_front = ((c3*desired_moments[1]) - (c1*desired_moments[2]) + (c2*desired_thrust))/denom_1;
    // float omega_right = ((c4*desired_moments[2]) - (c3*desired_moments[0]) + (c5*desired_thrust))/denom_2;
    // float omega_left = ((c3*desired_moments[0]) + (c4*desired_moments[2]) + (c5*desired_thrust))/denom_2;
    // float omega_back = -((c3*desired_moments[1]) + (c1*desired_moments[2]) - (c2*desired_thrust))/denom_1;
    float omega_front = desired_base_speed + desired_delta[0] - desired_delta[2];
    float omega_right = desired_base_speed - desired_delta[1] + desired_delta[2];
    float omega_left = desired_base_speed + desired_delta[1] + desired_delta[2];
    float omega_back = desired_base_speed - desired_delta[0] - desired_delta[2];


    float omega[4] = {omega_back, omega_left, omega_right, omega_front};

    // Convert motor speeds in rad/s to throttle commands between 0-1000
    // Gain to convert rad/s to throttle command
    // omega = KT*i/k_t -> build mapping of i to throttle
    // Motor torque constant in N-m/A (1/KV)
    const float KT = 0.006366198;
    // Gain to convert (rad/s)^2 to A
    const float K = KT/k_t;
    // This array holds the current produced by the motor for each 10% of throttle, starting at 0%
    const float Current[11] = {0.0, 0.1, 0.7, 2.0, 4.1, 7.2, 10.9, 15.4, 20.5, 25.9, 31.8};
    uint8_t first;
    uint8_t last;
    uint8_t middle;
    float I;
    for (uint8_t i = 0; i < 4; i++){
        first = 0;
        last = 10;
        I = omega[i]/K;
        if (I < 0.0){
            motor_throttles[i] = 0;
            continue;
        }
        if (I > Current[10]){
            motor_throttles[i] = 1000;
            continue;
        }
        while(1){
            middle = (last - first)/2 + first;
            if (middle == first){
                break;
            }
            if (I < Current[middle]){
                last = middle;
                continue;
            }
            if (I > Current[middle]){
                first = middle;
            }
        }
        float temp = ((I - Current[first])/(Current[last]-Current[first]))*100;
        motor_throttles[i] = first*100 + (uint16_t)temp;
        if (motor_throttles[i]  < 310) motor_throttles[i] = 0;
    }
}