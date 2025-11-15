#define _USE_MATH_DEFINES
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "Controllers_p32.h"
#include "Guidance_p32.h"
#include "Navigation_p32.h"
#include "IMU_p32.h"
#include "Barometer_p32.h"
#include "Global_Variables_p32.h"
#include "Least_Squares_p32.h"
#include "External_Interface.h"
#include "Lora_p32.h"

static Control_Variables Output;
static Time Outer_Loop_Timelast;
static Time Middle_Loop_Timelast;
static Time Inner_Loop_Timelast;
static Time Outer_Loop_Timelast_M;
static Time Inner_Loop_Timelast_M;
static double velocity_vector[3];
static double acceleration_vector[3];
static double Euler_dot_ref[3];
static double e_ddot_int;

void Initialize_Controllers(){
    memset(&Outer_Loop_Timelast, 0, sizeof(Outer_Loop_Timelast));
    memset(&Middle_Loop_Timelast, 0, sizeof(Middle_Loop_Timelast));
    memset(&Inner_Loop_Timelast, 0, sizeof(Inner_Loop_Timelast));
    memset(&Outer_Loop_Timelast_M, 0, sizeof(Outer_Loop_Timelast_M));
    memset(&Inner_Loop_Timelast_M, 0, sizeof(Inner_Loop_Timelast_M));
    memset(&velocity_vector, 0, sizeof(velocity_vector));
    memset(&acceleration_vector, 0, sizeof(acceleration_vector));
    memset(&Euler_dot_ref, 0, sizeof(Euler_dot_ref));
    e_ddot_int = 0.0;
    memset(&Output, 0, sizeof(Output));
}

void Run_Controllers(){
    const double max_angle = M_PI/4.0;
    
    if ((fabs(Air_Filter_data(3)) > max_angle) || (fabs(Air_Filter_data(4))) > max_angle){
        memset(&Output, 0, sizeof(Output));
        memcpy(e_throttle_commands, Output.Throttles, sizeof(e_throttle_commands));
        Manage_FC_Status(Standby);
        return;
    }
    
    if (Inhibit_Motors()){
        return;
    }

    Thrust_Control();

    Moment_Control();

}

void Thrust_Control(){
    const Time Outer_Loop_Update_Time = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/2};
    const Time Middle_Loop_Update_Time = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/20};
    const Time Inner_Loop_Update_Time = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/200};

    // Outer loop controls position (height) by setting desired velocity vector
    if (Compare_And_Update(Current_Time(), Outer_Loop_Update_Time, &Outer_Loop_Timelast)){
        double h = Altitude_Filter_data(0);
        double h_ref = Reference_Altitude();
        velocity_vector[2] = h_ref - h;
        Saturate(&velocity_vector[2], -0.5, 0.5);
    }
    // Middle loop controls velocity to align with desired vector
    if (Compare_And_Update(Current_Time(), Middle_Loop_Update_Time, &Middle_Loop_Timelast)){
        double h_dot = Altitude_Filter_data(1);
        double e_dot = velocity_vector[2] - h_dot;
        acceleration_vector[2] = 0.5 * e_dot;
    }
    // Innermost loop controls acceleration 
    if (Compare_And_Update(Current_Time(), Inner_Loop_Update_Time, &Inner_Loop_Timelast)){
        double phi = Air_Filter_data(3);
        double theta = Air_Filter_data(4);
        double Accel_Up = -(-sin(theta)*IMU_Acceleration(0) + sin(phi)*cos(theta)*IMU_Acceleration(1) + cos(phi)*cos(theta)*IMU_Acceleration(2) + 1.0);
    
        double e_ddot = acceleration_vector[2] - Accel_Up;
        e_ddot_int += 0.05*e_ddot;
        Saturate(&e_ddot_int, -0.0, 2.0);
        Output.Thrust = Output.Thrust * 0.9 + (e_ddot + e_ddot_int) * g_gravity * g_mass * 0.1;
    }
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
    const Time Outer_Loop_Update_Time = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/20};
    const Time Inner_Loop_Update_Time = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/200};
    const double moment_split = 0.75;
	const double k_f = 0.000001;
	const double k_t = 0.000000011;
	const double length_f_b = 0.089;
	const double length_l_r = 0.095;

    // Outer loop controls angle 
    if (Compare_And_Update(Current_Time(), Outer_Loop_Update_Time, &Outer_Loop_Timelast_M)){
        double e[3];
        for (uint8_t i = 0; i < 3; i++){
            e[i] = Reference_Euler(i) - Air_Filter_data(3+i);
            Euler_dot_ref[i] = e[i];
            Saturate(&Euler_dot_ref[i], -M_PI, M_PI);
        }
    }
    // Innermost loop controls angular rate
    if (Compare_And_Update(Current_Time(), Inner_Loop_Update_Time, &Inner_Loop_Timelast_M)){
        double Euler_Frame_Moments[3];
        double Euler_dot[3];
        double Max_moment[3];
        // Because motors cannot run backwards, there is a saturation point for how much moment can be applied that is a function of the thrust
        // Compute the omega (speed^2) expected of each motor to reach this thrust, this becomes maximum control authority for euler control

        Max_moment[0] = (Output.Thrust * length_l_r)/2.0;
        Max_moment[0] *= moment_split;
        Max_moment[1] = (Output.Thrust * length_f_b)/2.0;
        Max_moment[1] *= moment_split;
        Max_moment[2] = Output.Thrust*(k_t / k_f);
        Max_moment[2] *= (1.0 - moment_split);

        for (uint8_t i = 0; i < 3; i++){
            Euler_dot[i] = Air_Filter_x_Dot(3+i);
            Euler_Frame_Moments[i] = 0.1 * (Euler_dot_ref[i] - Euler_dot[i]);
            Saturate(&Euler_Frame_Moments[i], -Max_moment[i], Max_moment[i]);
        }
        Output.Moments[0] = Euler_Frame_Moments[0] - sin(Air_Filter_data(4))*Euler_Frame_Moments[2];
        Output.Moments[1] = cos(Air_Filter_data(3))*Euler_Frame_Moments[1] + sin(Air_Filter_data(3))*cos(Air_Filter_data(4))*Euler_Frame_Moments[2];
        Output.Moments[2] = -sin(Air_Filter_data(3))*Euler_Frame_Moments[1] + cos(Air_Filter_data(3))*cos(Air_Filter_data(4))*Euler_Frame_Moments[2];
        Set_throttles();
    }

}

void Set_throttles(){
    // -> Front left (ESC 2, index 0) produces positive pitching torque, positive rolling torque, and negative yawing torque
    // -> Front right (ESC 4, index 1) produces positive pitching torque, negative rolling torque, and positive yawing torque
    // -> Back left (ESC 1, index 2) produces negative pitching torque, positive rolling torque, and positive yawing torque
    // -> Back right (ESC 3, index 3) produces negative pitching torque, negative rolling torque, and negative yawing torque
    
	const double k_f = 0.000001;
	const double k_t = 0.000000011;
	const double length_f_b = 0.089;
	const double length_l_r = 0.095;

    const double denom = 4.0 * k_f * k_t * length_f_b * length_l_r;
    const double c1 = k_t * length_f_b;
    const double c2 = k_t * length_l_r;
    const double c3 = k_f * length_f_b * length_l_r;
    const double c4 = k_t * length_f_b * length_l_r;

    double T = Output.Thrust;
    double Mx = Output.Moments[0];
    double My = Output.Moments[1];
    double Mz = Output.Moments[2];

    double omega_front_left  = ( c1*Mx - c2*My + c3*Mz + c4*T)/denom;
    double omega_front_right = ( c1*Mx + c2*My - c3*Mz + c4*T)/denom;
    double omega_back_left   = (-c1*Mx - c2*My - c3*Mz + c4*T)/denom;
    double omega_back_right  = (-c1*Mx + c2*My + c3*Mz + c4*T)/denom;

    double omega[4] = {omega_front_left, omega_front_right, omega_back_left, omega_back_right};

    const double rpm2rads = (M_PI/30.0);
    // Solve backwards for throttle by using the equation w = Throt*slope + offset
    // Throt = (w - offset)/slope
    // If the throttle is less than 350 (35%), use:
    const double slope_low  = 23.2 * rpm2rads;
    const double offset_low = 116.0 * rpm2rads;
    // Corresponds to about 1200 rpm per 5% throttle
    // If the throttle is greater than 350, use:
    const double slope_high  = 21.1 * rpm2rads;
    const double offset_high = 821.0 * rpm2rads;
    // Corresponds to about 1000 rpm per 5% throttle
    const double crossover_speed = (slope_low*350.0 + offset_low)*rpm2rads;

    double throttle_dp;
    double omega_root;
    for (uint8_t i = 0; i < 4; i++){
        if (omega[i] <= 0.0){
            Output.Throttles[i] = 0;
            continue;
        }

        omega_root = sqrt(omega[i]);
        if (omega_root <= crossover_speed){
            throttle_dp = (omega_root - offset_low) / slope_low;
        }
        else{
            throttle_dp = (omega_root - offset_high) / slope_high;
        }

        if (throttle_dp > 1000.0){
            throttle_dp = 1000.0;
        }

        Output.Throttles[i] = (uint16_t)throttle_dp;
    }
    // Write throttle commands into interface
    memcpy(e_throttle_commands, Output.Throttles, sizeof(e_throttle_commands));
}