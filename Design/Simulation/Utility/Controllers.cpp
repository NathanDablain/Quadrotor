#include "Controllers.h"

Drone_Constants Initialize_Drone_Constants(){
    Drone_Constants Constants = {
	.k_f = 0.000001,
	.k_t = 0.000000011,
	.length_f_b =  0.127,
	.length_l_r = 0.125, 
	.I = {0.0018, 0.00356, 0.00208},
	.mass = 0.5885,
	.KT = 0.006366198,
	.K = 0.006366198/0.000000011,
	.Current = {0.0, 0.1, 0.7, 2.0, 4.1, 7.2, 10.9, 15.4, 20.5, 25.9, 31.8}};
		
	return Constants;
}

float Altitude_Control(float h, float h_ref, const Drone_Constants *Constants){
    // u = -K*x -> x = h_int, h, h_dot
    const float d_t = 0.01;
    const float c1 = 0.9;
    const float c2 = 1.0-c1;
    const float K[2] = {3.0, 5.0};
    const float K_int = 0.002;

    static float e_int;
    static float h_last;
    static float h_dot_last;

    e_int += (h_ref-h);
    float h_dot = h_dot_last*c1 + (c2*(h-h_last)/d_t);
    h_dot_last = h_dot;
    h_last = h;
    float u = -K[0]*(h-h_ref) - K[1]*h_dot + K_int*e_int + 9.81;
    float thrust = Constants->mass*u;

    return thrust; 
}

void Euler_Control(float Current_Euler[3], float Commanded_Euler[3], float desired_moments[3], float thrust, const Drone_Constants *Constants){
    const float d_t = 0.0025;
    const float K[3][2] = {{40, 40},{40, 40},{4, 8}};
    const float K_int = 0.05;
    const float IIR_c1 = 0.9;
    const float IIR_c2 = 1.0 - IIR_c1;
    const float moment_split = 0.75;
    // The maximum moments allowed should be a function of the thrust commanded.
    // This is to precent sending the motors into saturation because they can only rotate one direction
    static float Euler_last[3];
    static float Euler_dot_last[3];
    static float e_int[3];
    static uint8_t Saturation_Flag[3];
    
    float max_moment_phi = (thrust*Constants->length_l_r)/2.0;
    max_moment_phi *= moment_split;
    float max_moment_theta = (thrust*Constants->length_f_b)/2.0;
    max_moment_theta *= moment_split;
    float max_moment_psi = thrust*(Constants->k_t/Constants->k_f);
    max_moment_psi *= (1.0-moment_split);


    for (uint8_t i = 0; i < 3; i++){
        float e = Commanded_Euler[i] - Current_Euler[i];
		if ((Saturation_Flag[i] == 0) || ((Saturation_Flag[i] == 1) && (e < 0)) || ((Saturation_Flag[i] == 2) && (e > 0))){
			e_int[i] += e;
		}
        float Euler_dot = Euler_dot_last[i]*IIR_c1 + ((Current_Euler[i]-Euler_last[i])/d_t)*IIR_c2;
        Euler_last[i] = Current_Euler[i];
        Euler_dot_last[i] = Euler_dot;
        float u = K[i][0]*e - K[i][1]*Euler_dot + K_int*e_int[i];
        desired_moments[i] = u*Constants->I[i];
    }

    Saturate(desired_moments[0], max_moment_phi, -max_moment_phi, Saturation_Flag[0]);
    Saturate(desired_moments[1], max_moment_theta, -max_moment_theta, Saturation_Flag[1]);
    Saturate(desired_moments[2], max_moment_psi, -max_moment_psi, Saturation_Flag[2]);
}

void Saturate(float &desired_moment, float max, float min, uint8_t &Saturation_Flag){
    if (desired_moment > max){
        desired_moment = max;
        Saturation_Flag = 1;
    }
    else if (desired_moment < min){
        desired_moment = min;
        Saturation_Flag = 2;
    }
    else {
        Saturation_Flag = 0;
    }
}

void Set_throttles(uint16_t motor_throttles[4], float desired_thrust, float desired_moments[3], const Drone_Constants *Constants){
    // -> Back motor (0) produces negative pitching torque and positive yawing torque
    // -> Left motor (1) produces positive rolling torque and negative yawing torque
    // -> Right motor (2) produces negative rolling torque and negative yawing torque
    // -> Front motor (3) produces positive pitching torque and positive yawing torque

    const float denom_1 = 4.0 * Constants->k_f * Constants->k_t * Constants->length_f_b;
    const float denom_2 = 4.0 * Constants->k_f * Constants->k_t * Constants->length_l_r;
    const float c1 = Constants->k_f * Constants->length_f_b;
    const float c2 = Constants->k_t * Constants->length_f_b;
    const float c3 = 2.0 * Constants->k_t;
    const float c4 = Constants->k_f * Constants->length_l_r;
    const float c5 = Constants->k_t * Constants->length_l_r;

    // w_f: (2*My*kt + Mz*kf*lfb + T*kt*lfb)/(4*kf*kt*lfb)
    // w_r: (-Mz*kf*lrl - 2*Mx*kt + T*kt*lrl)/(4*kf*kt*lrl)
    // w_l: (2*Mx*kt - Mz*kf*lrl + T*kt*lrl)/(4*kf*kt*lrl)
    // w_b: (-2*My*kt + Mz*kf*lfb + T*kt*lfb)/(4*kf*kt*lfb)

    float omega_front = ((c3*desired_moments[1]) + (c1*desired_moments[2]) + (c2*desired_thrust))/denom_1;
    float omega_right = ((-c4*desired_moments[2]) - (c3*desired_moments[0]) + (c5*desired_thrust))/denom_2;
    float omega_left = ((c3*desired_moments[0]) - (c4*desired_moments[2]) + (c5*desired_thrust))/denom_2;
    float omega_back = (-(c3*desired_moments[1]) + (c1*desired_moments[2]) + (c2*desired_thrust))/denom_1;

    float omega[4] = {omega_back, omega_left, omega_right, omega_front};

    // Convert motor speeds in rad/s to throttle commands between 0-1000
    // omega = KT*i/k_t -> build mapping of i to throttle
   
    uint8_t first;
    uint8_t last;
    uint8_t middle;
    float I;
    for (uint8_t i = 0; i < 4; i++){
        first = 0;
        last = 10;
        I = omega[i]/Constants->K;
        if (I < 0.0){
            motor_throttles[i] = 0;
            continue;
        }
        if (I > Constants->Current[10]){
            motor_throttles[i] = 1000;
            continue;
        }
        while(1){
            middle = (last - first)/2 + first;
            if (middle == first){
                break;
            }
            if (I < Constants->Current[middle]){
                last = middle;
                continue;
            }
            if (I > Constants->Current[middle]){
                first = middle;
            }
        }
        float temp = ((I - Constants->Current[first])/(Constants->Current[last]-Constants->Current[first]))*100;
        motor_throttles[i] = first*100 + (uint16_t)temp;
    }
}

void Safety_Check(uint16_t motor_throttles[4], States *Drone, FC_Status *Flight_Controller_Status){
    uint8_t safety_switch = 0;
	
	if (fabs(Drone->Euler[0]) > MOTOR_CUTOFF_ANGLE) safety_switch = 1;
	if (fabs(Drone->Euler[1]) > MOTOR_CUTOFF_ANGLE) safety_switch = 1;
	if (fabs(Drone->Position_NED[2]) > MOTOR_CUTOFF_ALTITUDE) safety_switch = 1;
	
	if (safety_switch){
        memset(motor_throttles, 0, 8);
		*Flight_Controller_Status = Standby;
	}
}

void Run_Guidance(Reference *Desired_States, Reference *Commanded_States){
	// IIR to prevent large jumps in reference states
	const float c1 = 0.99;
	const float c2 = 1.0 - c1;
	for (uint8_t i = 0; i < 3; i++){
		Commanded_States->Euler[i] = Commanded_States->Euler[i]*c1 + Desired_States->Euler[i]*c2;
		Commanded_States->Position_NED[i] = Commanded_States->Position_NED[i]*c1 + Desired_States->Position_NED[i]*c2;
	}
}