#define _USE_MATH_DEFINES
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "Controllers_p32.h"
#include "Guidance_p32.h"
#include "IMU_p32.h"
#include "Barometer_p32.h"
#include "Global_Variables_p32.h"

static Control_Variables Output;

void Initialize_Controllers(){
    memset(&Output, 0, sizeof(Output));
    Output.Mass_Modifier = 1.0;
}

void Thrust_Control(){
    // u = -K*x -> x = h_int, h, h_dot
    const double expected_takeoff_thrust = g_gravity*g_mass;
    double K[3] = {1.0, 0.0005, 0.8};
    double minimum_thrust = 0.5*g_gravity*g_mass*Output.Mass_Modifier;
    double maximum_thrust = 2.0*g_gravity*g_mass*Output.Mass_Modifier;
    const double maximum_integated_error = 0.5*g_gravity/K[2];
    const double minimum_integrated_error = -0.5*g_gravity/K[2];
    const double minimum_error = -5.0;
    const double maximum_error = 5.0;

    double h = Filter_data(0, 2);
    double h_dot = Filter_data(1, 2);
    double h_ref = Reference_Altitude();
    double e = h_ref - h;
    double e_dot = -h_dot;
    Saturate(&e, minimum_error, maximum_error);

    switch(Get_Guidance_State()){
        case Taking_Off:
            Output.Thrust_e_int += 0.02*e;
            Output.Thrust = Output.Thrust_e_int;
            return;

        case Climbing:
            if (Output.Hover_Thrust == 0.0){
                Output.Hover_Thrust = 0.6*Output.Thrust;
                Output.Thrust_e_int = 0.0;
                Output.Mass_Modifier = Output.Hover_Thrust/expected_takeoff_thrust;
            }
            Output.Thrust_e_int += K[1]*e;
            Output.Thrust = Output.Hover_Thrust + K[0]*e + Output.Thrust_e_int + K[2]*e_dot;
            break;

        case Hovering:
            Output.Thrust_e_int += K[1]*e;
            Output.Thrust = Output.Hover_Thrust + K[0]*e + Output.Thrust_e_int + K[2]*e_dot;
            break;
        case Descending:
            K[2] = 2.5;
            Output.Thrust_e_int += K[1]*e;
            Output.Thrust = Output.Hover_Thrust + K[0]*e + Output.Thrust_e_int + K[2]*e_dot;
            break;

        case Landed:
            Output.Thrust = 0.0;
            break;

    }
    Saturate(&Output.Thrust_e_int, minimum_integrated_error, maximum_integated_error);
    Saturate(&Output.Thrust, minimum_thrust, maximum_thrust);
}

void Saturate(double *Value, double Min, double Max){
    if (*Value < Min){
        *Value = Min;
    }
    else if (*Value > Max){
        *Value = Max;
    }
}

void Moment_Control(){
    const double K[3] = {0.8, 1.0, 0.001};
    double e[3];
    double Euler_Frame_Moments[3];
    if (Inhibit_Motors()){
        memset(Output.Moments, 0, sizeof(Output.Moments));
        Set_throttles();
        return;
    }
    
    e[0] = Reference_Euler(0) - Filter_data(3, 1);
    e[1] = Reference_Euler(1) - Filter_data(4, 1);
    e[2] = Reference_Euler(2) - Filter_data(5, 1);

    Euler_Frame_Moments[0] = K[0]*e[0] - K[1]*Air_Filter_x_Dot(3) - K[2]*Output.Moment_e_int[0];
    Euler_Frame_Moments[1] = K[0]*e[1] - K[1]*Air_Filter_x_Dot(4) - K[2]*Output.Moment_e_int[1];
    Euler_Frame_Moments[2] = K[0]*e[2] - K[1]*Air_Filter_x_Dot(5) - K[2]*Output.Moment_e_int[2];
    Euler_Frame_Moments[2] *= 0.1;

    Output.Moment_e_int[0] += 0.0001*e[0];
    Output.Moment_e_int[1] += 0.0001*e[1];
    Output.Moment_e_int[2] += 0.0001*e[2];

    // These moments are in the euler frame, the throttles are mapped in the drone body frame

    Output.Moments[0] = Euler_Frame_Moments[0] - sin(Filter_data(4, 1))*Euler_Frame_Moments[2];
    Output.Moments[1] = cos(Filter_data(3, 1))*Euler_Frame_Moments[1] + sin(Filter_data(3, 1))*cos(Filter_data(4, 1))*Euler_Frame_Moments[2];
    Output.Moments[2] = -sin(Filter_data(3, 1))*Euler_Frame_Moments[1] + cos(Filter_data(3, 1))*cos(Filter_data(4, 1))*Euler_Frame_Moments[2];

    Set_throttles();
}

void Set_throttles(){
    // -> Back motor (0) produces negative pitching torque and positive yawing torque
    // -> Left motor (1) produces positive rolling torque and negative yawing torque
    // -> Right motor (2) produces negative rolling torque and negative yawing torque
    // -> Front motor (3) produces positive pitching torque and positive yawing torque
	const double k_f = 0.000001;
	const double k_t = 0.000000011;
	const double length_f_b =  0.127;
	const double length_l_r = 0.125; 

    const double denom_1 = 4.0 * k_f * k_t * length_f_b;
    const double denom_2 = 4.0 * k_f * k_t * length_l_r;
    const double c1 = k_f * length_f_b;
    const double c2 = k_t * length_f_b;
    const double c3 = 2.0 * k_t;
    const double c4 = k_f * length_l_r;
    const double c5 = k_t * length_l_r;

    // w_f: (2*My*kt + Mz*kf*lfb + T*kt*lfb)/(4*kf*kt*lfb)
    // w_r: (-Mz*kf*lrl - 2*Mx*kt + T*kt*lrl)/(4*kf*kt*lrl)
    // w_l: (2*Mx*kt - Mz*kf*lrl + T*kt*lrl)/(4*kf*kt*lrl)
    // w_b: (-2*My*kt + Mz*kf*lfb + T*kt*lfb)/(4*kf*kt*lfb)

    double omega_front = ((c3*Output.Moments[1]) + (c1*Output.Moments[2]) + (c2*Output.Thrust))/denom_1;
    double omega_right = ((-c4*Output.Moments[2]) - (c3*Output.Moments[0]) + (c5*Output.Thrust))/denom_2;
    double omega_left = ((c3*Output.Moments[0]) - (c4*Output.Moments[2]) + (c5*Output.Thrust))/denom_2;
    double omega_back = (-(c3*Output.Moments[1]) + (c1*Output.Moments[2]) + (c2*Output.Thrust))/denom_1;

    double omega[4] = {omega_back, omega_left, omega_right, omega_front};

    // Solve backwards for throttle by using the equation w = Throt*228 + 316
    // -> Throt = (w - 316)/228
    const double rpm2rads = (M_PI/30.0);
    const double motor_c1 = 316.0*rpm2rads;
    const double motor_c2 = 228.0*rpm2rads;
    double temp;
    for (uint8_t i = 0; i < 4; i++){
        if (omega[i] <= 0.0){
            Output.Throttles[i] = 0;
            continue;
        }
        temp = ((sqrt(omega[i]) - motor_c1)/motor_c2)*10.0;
        if (temp > 1000) temp = 1000;
        Output.Throttles[i] = (uint16_t)temp;
    }
}

// cpp wrapper functions

void Initialize_Controllers_cpp(){
    Initialize_Controllers();
}

void Thrust_Control_cpp(){
    Thrust_Control();
}

void Moment_Control_cpp(){
    Moment_Control();
}

void Read_Throttles_cpp(uint16_t Throttles_cpp[4]){
    Throttles_cpp[0] = Output.Throttles[0];
    Throttles_cpp[1] = Output.Throttles[1];
    Throttles_cpp[2] = Output.Throttles[2];
    Throttles_cpp[3] = Output.Throttles[3];
}