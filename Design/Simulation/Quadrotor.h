#pragma once

#include <vector>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include "Sim_Types.h"
#include "Sim_Time.h"
#include "Linear_Algebra.h"
#include "Motor.h"
#include "Environment.h"
#include "Coordinate_Frames.h"
#include "Gaussian.h"
#include "Barometer.h"
#include "IMU.h"
#include "Magnetometer.h"


#define STANDARD_WIDTH 20
#define LOG_DATA(data,log_name) (log_name << setw(STANDARD_WIDTH) << data)
#define LOG_VEC3(vec3,log_name) (log_name << setw(STANDARD_WIDTH) << vec3.data[0] << setw(STANDARD_WIDTH) << vec3.data[1] << setw(STANDARD_WIDTH) << vec3.data[2])
#define LOG_ARR3(arr3,log_name) (log_name << setw(STANDARD_WIDTH) << arr3[0] << setw(STANDARD_WIDTH) << arr3[1] << setw(STANDARD_WIDTH) << arr3[2])

extern "C" {
    void Reset_External_Interface();
    void Initialize_p32(bool log_flag);
    void Execute_p32();
}

class Quadrotor{
    private:
        std::ofstream log_sim;
        std::ofstream log_pic;
        bool log_flag = true;
        bool plot_flag = true;
        bool error_flag = false;
        // The following parameters can be varied with montecarlo seeds, specify the mean value and the expected variance
        // Actual mass of drone in (kg)
        double mass;
        // Distance from front and back motor thrust vectors to drone center of gravity in (m)
        double length_f_b;
        // Distance from left and right motor thrust vectors to drone center of gravity in (m)
        double length_l_r;
        // Propeller parameters
        double mu;
        // Initial conditions
        Vec3 Initial_Euler;
        // Step time for simulation
        Sim_Time sim_dt;
        // Final time of simulation
        Sim_Time sim_tf;
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
        // Body linear acceleration (m/s^2), includes ficticious coriolis effect
        Vec3 a;
        // Body angular velocity (rad/s)
        Vec3 w;
        // Body angular velocity (deg/s)
        Vec3 w_deg_s;
        // Euler angles (rad)
        Vec3 Euler;
        // Euler angles (degrees)
        Vec3 Euler_deg;
        // NED Position (meters)
        Vec3 Position_NED;
        // NED Velocity (meters/s)
        Vec3 Velocity_NED;
        // NED to body quaternion
        Vec4 q;
        // Model of BLDC motor and propeller
        Motor Motors[4]; // Back (CCW), Left (CW), Front (CCW), Right (CW)
        uint8_t calibration_phase;
        Gaussian Moment_noise_gauss;
        Gaussian Force_noise_guass;
        Sim_Time Time_last_log;
        Sim_Time Time_last_update_motors;
        // LORA
        Sim_Time last_transmit_time;
        uint8_t Lora_ID_index;
        float Lora_Desired_North = 0.0;
        float Lora_Desired_East = 0.0;
        float Lora_Desired_Altitude = 0.0;
        float Lora_Pressure_Altitude = 0.0;
        FC_Status Lora_Desired_Status = Standby;
    public:
        // Current time in simulation
        Sim_Time sim_t;
        FC_Status inbound_Flight_Controller_Status = Standby;
        bool Successful_Landing = false;
        // LPS22H Barometer
        Barometer barometer;
        // LIS2MDL Magnetometer
        Magnetometer magnetometer;
        // LSM6DS3TR IMU
        IMU imu;
        double Control_errors[6];
        double Navigation_errors[6];
        Quadrotor(Sim_Time Sim_dt, Sim_Time Sim_tf);
        void Run_Sensors(Environment &env);
        void Update_drone_states();
        void Update_drone_forces_moments(Environment &env);
        void Run_sim();
        void Run_Ground_Controller();
        void Log_data(Environment &env);
        std::array<double, 13> Differential_equation_momentum(std::array<double, 13> x_in);
        void Set_Monte_Carlo_Data(Monte_Carlo_Data MC_Data);

};