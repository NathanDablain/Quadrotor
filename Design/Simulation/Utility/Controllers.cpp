#include "Controllers.h"

float Height_MRAC(States &MCU, float h_ref){
    const float d_t = 0.025;
    // Reference Model
    // h and h_dot
    static float x_ref[2];
    float x_ref_dot[2];
    // Natural frequency and damping ratio for reference model
    const float w_n = 1.0;
    const float zeta = 0.707;
    const float c1 = 2.0*zeta*w_n;
    const float c2 = pow(w_n, 2);
    float h = -MCU.Position_NED[2];
    
    x_ref_dot[0] = x_ref[1];
    x_ref_dot[1] = -c1*x_ref[0] - c2*x_ref[1] + c1*h_ref;
    x_ref[0] += (x_ref_dot[0]*d_t);
    x_ref[1] += (x_ref_dot[1]*d_t);

    // Update Gains
    static float h_last;
    float e[2];
    e[0] = x_ref[0] - h;
    float h_dot = (h - h_last)/d_t;
    h_last = h;
    e[1] = x_ref[1] - h_dot;

    // This is actually a 2x2 matrix but P_12 = P_21 so only store that value once as P_lyap[1]
    static float k_x[2], k_r, w;
    const float P_lyap[3] = {36.2143, -0.5, 50.7070};
    const float gamma_x[2] = {0.2, 10.0};
    float k_x_dot[2];

    float phi_x = cos(MCU.Euler[0])*cos(MCU.Euler[1]);
    k_x_dot[0] = gamma_x[0]*-MCU.Position_NED[2]*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);
    k_x_dot[1] = gamma_x[1]*h_dot*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);
    float k_r_dot = h_ref*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);
    float w_dot = phi_x*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);

    // Integrate controller gains
    k_x[0] += (k_x_dot[0]*d_t);
    k_x[1] += (k_x_dot[1]*d_t);
    k_r += (k_r_dot*d_t);
    w += (w_dot*d_t);

    // Update Thrust
    float thrust = k_x[0]*h + k_x[1]*h_dot + k_r*h_ref - w*phi_x;
    if (thrust > MAX_THRUST){
        thrust = MAX_THRUST;
    }
    else if (thrust < 0.0){
        thrust = 0.0;
    }
    return thrust;
}

void Set_throttles(uint16_t motor_throttles[4], float desired_thrust, float desired_moments[3]){
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
    const float c2 = k_t*length_f_b;
    const float c3 = k_t*length_l_r;
    const float denom_1 = (2.0*k_f*length_f_b*(c2 + c3));
    const float denom_2 = (2.0*k_f*length_l_r*(c2 + c3));
    const float c1 = c2*length_l_r;
    const float c4 = k_f*length_f_b;
    const float c5 = k_f*length_l_r;

    float omega_front = (desired_moments[1]*c2 - desired_moments[2]*c4 + desired_moments[1]*c3 + desired_thrust*c1)/denom_1;
    float omega_right = -(desired_moments[0]*c2 - desired_moments[2]*c5 + desired_moments[0]*c3 - desired_thrust*c1)/denom_2;
    float omega_left = (desired_moments[0]*c2 + desired_moments[2]*c5 + desired_moments[0]*c3 + desired_thrust*c1)/denom_2;
    float omega_back = -(desired_moments[2]*c4 + desired_moments[1]*c2 + desired_moments[1]*c3 - desired_thrust*c1)/denom_1;

    float w_front = (omega_front > 0)?(sqrt(omega_front)):(0.0);
    float w_right = (omega_right > 0)?(sqrt(omega_right)):(0.0);
    float w_left = (omega_left > 0)?(sqrt(omega_left)):(0.0);
    float w_back = (omega_back > 0)?(sqrt(omega_back)):(0.0);
    // Convert motor speeds in rad/s to throttle commands between 0-1000
    // Gain to convert rad/s to throttle command
    const float K = 0.2344;
    motor_throttles[0] = (uint16_t)(w_back*K);
    motor_throttles[1] = (uint16_t)(w_left*K);
    motor_throttles[2] = (uint16_t)(w_right*K);
    motor_throttles[3] = (uint16_t)(w_front*K);
}