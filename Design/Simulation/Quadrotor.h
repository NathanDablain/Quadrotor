#pragma once

#include <vector>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <iomanip>
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
#define LOG_SIM(data) (log_sim << setw(STANDARD_WIDTH) << data)
#define LOG_MCU(data) (log_mcu << setw(STANDARD_WIDTH) << data)

class Quadrotor{
    private:
        ofstream log_sim;
        ofstream log_mcu;
        bool log_flag = true;
        // Actual mass of drone in (kg)
        double mass = 0.441;
        // Distance from front and back motor thrust vectors to drone center of gravity in (m)
        double length_f_b =  0.117;
        // Distance from left and right motor thrust vectors to drone center of gravity in (m)
        double length_l_r = 0.1205;
        // Actual mass moments of drone in (kg-m^2)
        double Ixx = 0.00149;
        double Iyy = 0.00262;
        double Izz = 0.00149;
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
        // Body angular velocity
        Vec3 w;
        // Euler angles
        Vec3 Euler;
        // NED Position
        Vec3 Position_NED;
        // NED to body quaternion
        Vec q;
        // Model of BLDC motor and propeller
        Motor Motors[4]; // Back (CW), Left (CCW), Front (CW), Right (CCW)
        
        MCU AVR128DB48;
    public:
        Quadrotor(Sim_Time Sim_dt, Sim_Time Sim_tf);
        void Run_Sensors(Environment &env);
        void Manage_FC_Status();
        void Update_drone_states();
        void Update_drone_forces_moments(Environment &env);
        void Run_sim();
        void Log_data(Environment &env);
        Vec Differential_equation_momentum(Vec x_in);
};