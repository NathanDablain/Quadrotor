#pragma once

#include <vector>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <iomanip>
#include <string.h>
#include "Sim_Types.h"
#include "Sim_Time.h"
#include "Linear_Algebra.h"
#include "Motor.h"
#include "Environment.h"
#include "Coordinate_Frames.h"
#include "Controllers.h"
#include "MCU.h"

using namespace std;

#define STANDARD_WIDTH 20
#define LOG_DATA(data,log_name) (log_name << setw(STANDARD_WIDTH) << data)
#define LOG_VEC3(vec3,log_name) (log_name << setw(STANDARD_WIDTH) << vec3.data[0] << setw(STANDARD_WIDTH) << vec3.data[1] << setw(STANDARD_WIDTH) << vec3.data[2])
#define LOG_ARR3(arr3,log_name) (log_name << setw(STANDARD_WIDTH) << arr3[0] << setw(STANDARD_WIDTH) << arr3[1] << setw(STANDARD_WIDTH) << arr3[2])

class Quadrotor{
    private:
        ofstream log_sim;
        ofstream log_mcu;
        bool log_flag = true;
        bool plot_flag = true;
        bool error_flag = false;
        // The following parameters can be varied with montecarlo seeds, specify the mean value and the expected variance
        // Actual mass of drone in (kg)
        double mass;
        const double mass_mean = 0.5885;
        const double mass_variance = mass_mean/10.0;
        // Distance from front and back motor thrust vectors to drone center of gravity in (m)
        double length_f_b;
        const double length_f_b_mean =  0.127;
        const double length_f_b_variance = length_f_b_mean/5.0;
        // Distance from left and right motor thrust vectors to drone center of gravity in (m)
        double length_l_r;
        const double length_l_r_mean = 0.125;
        const double length_l_r_variance = length_l_r_mean/5.0;
        // Actual mass moments of drone in (kg-m^2)
        const double Ixx = 0.0018;
        const double Ixx_variance = Ixx/2.0;
        const double Iyy = 0.00356;
        const double Iyy_variance = Iyy/2.0;
        const double Izz = 0.00208;
        const double Izz_variance = Izz/2.0;
        const double Ixy = 0.0000227;
        const double Ixy_variance = Ixy/2.0;
        const double Ixz = -0.0000012;
        const double Ixz_variance = Ixz/2.0;
        const double Iyz = 0.0000010;
        const double Iyz_variance = Iyz/2.0;
        // Step time for simulation
        Sim_Time sim_dt;
        // Final time of simulation
        Sim_Time sim_tf;
        // Current time in simulation
        Sim_Time sim_t;
        // Time to begin drone calibration
        Sim_Time cal_start_time;
        // Inertia matrix
        Mat3 inertia;
        // Forces applied to drone at each time step in North-East-Down coordinate frame (N)
        Vec3 Forces_NED;
        // Moments applied to drone at each time step in North-East-Down coordinate frame (N-m)
        Vec3 Moments_NED;
        // Forces applied to drone at each time step in Forward-Right-Down drone body coordinate frame (N)
        Vec3 Forces_Body;
        // Moments applied to drone at each time step in Forward-Right-Down drone body coordinate frame (N-m)
        Vec3 Moments_Body;
        // Body linear velocity
        Vec3 v;
        // Body angular velocity (rad/s)
        Vec3 w;
        // Body angular velocity (deg/s)
        Vec3 w_deg_s;
        // Euler angles (rad)
        Vec3 Euler;
        // Euler angles (degrees)
        Vec3 Euler_deg;
        // NED Position
        Vec3 Position_NED;
        // NED to body quaternion
        Vec q;
        // Model of BLDC motor and propeller
        Motor Motors[4]; // Back (CCW), Left (CW), Front (CCW), Right (CW)
        
    public:
        MCU AVR128DB48;

        double Control_errors[6];
        double Navigation_errors[6];
        Quadrotor(Sim_Time Sim_dt, Sim_Time Sim_tf, uint32_t mc_seed);
        void Run_Sensors(Environment &env);
        void Manage_FC_Status();
        void Update_drone_states();
        void Update_drone_forces_moments(Environment &env);
        void Run_sim();
        void Log_data(Environment &env);
        Vec Differential_equation_momentum(Vec x_in);
        void Calculate_errors();
        void Set_Monte_Carlo_Data(uint32_t seed);
};