#pragma once

#include <vector>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <iomanip>
#include "Linear_Algebra.hpp"
#include "Motor.h"
#include "Environment.h"
#include "Coordinate_Frames.h"

using namespace std;

#define LOG(data) (log << setw(20) << data)

#define BAR_SENS 1.0/40.96 // Pa/LSB
#define BAR_WINDOW_SIZE 16
#define BAR_TB 288.15 // Standard temperature at sea level (K)
#define BAR_G 9.8065 // Acceleration due to gravity (m/s^2)
#define BAR_LB -0.0065 // Standard temperature lapse rate (K/m)
#define BAR_PB 101325.0 // Standard static pressure at sea level (Pa)
#define BAR_R 8.31432 // Universal gas constant (J/(mol-K))S
#define BAR_M 0.0289644 // Molar mass of air (kg/mol)

#define MAG_WINDOW_SIZE 16

#define IMU_WINDOW_SIZE 8 // Size of FIR window
#define GYRO_SENS 500.0/32768.0
#define ACCEL_SENS 2.0/32768.0

float Height_Bar(uint32_t pressure_LSB);

struct States{
    float w[3];
    float Euler[3];
    float g_vec[3];
    float m_vec[3];
    float pressure_altitude;
    float Position_NED[3];
    int32_t Longitude;
    int32_t Latitude;
    uint32_t pressure; // Temporary
    float Position_ECEF[3];
};

class Quadrotor{
    private:
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
        // Step time for simulation in (s)
        double sim_dt;
        // Final time of simulation in (s)
        double sim_tf;
        // Current time in simulation in (s)
        double sim_t;
        // Time taken to calibrate drone in (s)
        double cal_t = 5.0;
        // Time taken to calibrate gyros in (s)
        double gyro_cal_t = 2.0;
        // Step time for MCU Real Time Clock in (s)
        double rtc_rate = 1.0;
        // Step time for MCU Timer/Counter B0 in (s)
        double tcb0_rate = 0.005;
        // Step time for MCU Timer/Counter B1 in (s)
        double tcb1_rate = 0.004807;
        // Step time for GPS transmission in (s)
        double gps_rate = 0.125;
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
        // Current state of the drone according to the MCU, should mirror the performance in src/Flight_Controller
        States MCU;
        // Desired state of the drone, should mirror its counterpart in src/Flight_Controller
        States Reference;
        // Model of BLDC motor and propeller
        Motor Motors[4]; // Back (CW), Left (CCW), Front (CW), Right (CCW)
    public:
        Quadrotor(double Sim_dt, double Sim_tf);
        void
            Update_drone_states(),
            Update_drone_forces_moments(double gravity, double ground_stiffness, double ground_damping),
            Run_MCU(Environment &env),
            Run_Motors(uint16_t throttles[4]),
            Observer(States &mcu),
            Run_sim(),
            Calibrate_sensors(Environment &env),
            Log_data(Environment &env),
            Read_GPS(States &mcu, Environment &env),
            Read_Bar(States &mcu, Environment &env),
            Read_Mag(States &mcu, Environment &env),
            Read_IMU(States &mcu, Environment &env),
            Read_LoRa(States &reference, Environment &env);
        Vec
            Differential_equation_momentum(Vec &x_in);
};