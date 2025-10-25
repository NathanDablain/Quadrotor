#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "time.h"
#include "controllers.h"
#include "navigation.h"
#include "guidance.h"
#include "imu.h"
#include "barometer.h"
#include "global_variables.h"
#include "motors.h"

typedef struct{
    double Sum;
    uint8_t counter;
    bool average_calculated;
} Average_Set;

static Control_Variables Output;
static Average_Set Acceleration_Sample_Points;
static Time Thrust_Controller_Timelast;
static Time Moment_Controller_Timelast;

void Initialize_Controllers(){
    memset(&Output, 0, sizeof(Output));
    memset(&Acceleration_Sample_Points, 0, sizeof(Acceleration_Sample_Points));
    Acceleration_Sample_Points.average_calculated = false;
    Output.Mass_Modifier = 1.0;
    Thrust_Controller_Timelast = Current_Time();
    Moment_Controller_Timelast = Current_Time();
}

void Run_Controllers(){
    const Time Thrust_Controller_Update_Time = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/50};
    const Time Moment_Controller_Update_Time = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/3000};
    
    if (Compare_And_Update(Current_Time(), Thrust_Controller_Update_Time, &Thrust_Controller_Timelast)){
        Thrust_Control();
    }
    if (Compare_And_Update(Current_Time(), Moment_Controller_Update_Time, &Moment_Controller_Timelast)){
        Moment_Control();
    }
}

void Thrust_Control(){
    // u = -K*x -> x = h_int, h, h_dot
    const double expected_takeoff_thrust = g_gravity*g_mass;
    double K[3] = {1.0, 0.01, 1.0}; // Proportional, Integral, Derivative
    double minimum_thrust = 0.5*g_gravity*g_mass*Output.Mass_Modifier;
    double maximum_thrust = 2.5*g_gravity*g_mass*Output.Mass_Modifier;
    const double maximum_integated_error = 0.5*g_gravity/K[1];
    const double minimum_integrated_error = -0.5*g_gravity/K[1];
    const double minimum_error = -5.0;
    const double maximum_error = 5.0;

    double h = Altitude_Filter_data(0);
    double h_dot = Altitude_Filter_data(1);
    double h_ref = Reference_Altitude();
    double e = h_ref - h;
    double e_dot = -h_dot;
    Saturate(&e, minimum_error, maximum_error);

    switch(Get_Guidance_State()){
        case Awaiting_Guidance:
            break;
        case Taking_Off:
            Output.Thrust_e_int += K[1]*e;
            Output.Thrust = g_gravity*g_mass + Output.Thrust_e_int;
            return;

        case Climbing:
            if (Acceleration_Sample_Points.counter != AVERAGE_SAMPLE_SIZE){
                Acceleration_Sample_Points.Sum += -IMU_Acceleration(2);
                Acceleration_Sample_Points.counter++;
            }
            else if (!Acceleration_Sample_Points.average_calculated){
                Output.Hover_Thrust =  Output.Thrust/(Acceleration_Sample_Points.Sum/((double)AVERAGE_SAMPLE_SIZE));
                Acceleration_Sample_Points.average_calculated = true;
                Output.Thrust_e_int = 0.0;
                Output.Mass_Modifier = Output.Hover_Thrust/expected_takeoff_thrust;
            }
            else{
                Output.Thrust_e_int += K[1]*e;
                Output.Thrust = Output.Hover_Thrust + K[0]*e + Output.Thrust_e_int + K[2]*e_dot;
            }
            break;

        case Hovering:
            Output.Thrust_e_int += K[1]*e;
            Output.Thrust = Output.Hover_Thrust + K[0]*e + Output.Thrust_e_int + K[2]*e_dot;
            break;
        case Descending:
            K[2] = 5.0;
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
    
    e[0] = Reference_Euler(0) - Air_Filter_data(3);
    e[1] = Reference_Euler(1) - Air_Filter_data(4);
    e[2] = Reference_Euler(2) - Air_Filter_data(5);

    Euler_Frame_Moments[0] = K[0]*e[0] - K[1]*Air_Filter_x_Dot(3) - K[2]*Output.Moment_e_int[0];
    Euler_Frame_Moments[1] = K[0]*e[1] - K[1]*Air_Filter_x_Dot(4) - K[2]*Output.Moment_e_int[1];
    Euler_Frame_Moments[2] = K[0]*e[2] - K[1]*Air_Filter_x_Dot(5) - K[2]*Output.Moment_e_int[2];
    Euler_Frame_Moments[2] *= 0.1;

    Output.Moment_e_int[0] += 0.0001*e[0];
    Output.Moment_e_int[1] += 0.0001*e[1];
    Output.Moment_e_int[2] += 0.0001*e[2];

    // These moments are in the euler frame, the throttles are mapped in the drone body frame

    Output.Moments[0] = Euler_Frame_Moments[0] - sin(Air_Filter_data(4))*Euler_Frame_Moments[2];
    Output.Moments[1] = cos(Air_Filter_data(3))*Euler_Frame_Moments[1] + sin(Air_Filter_data(3))*cos(Air_Filter_data(4))*Euler_Frame_Moments[2];
    Output.Moments[2] = -sin(Air_Filter_data(3))*Euler_Frame_Moments[1] + cos(Air_Filter_data(3))*cos(Air_Filter_data(4))*Euler_Frame_Moments[2];

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
    
    // Convert throttles to PWM and trigger update 
    Apply_Throttle_Batch(Output.Throttles);
}