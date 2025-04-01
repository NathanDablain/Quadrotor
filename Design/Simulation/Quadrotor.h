#pragma once

#include <vector>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <iomanip>
#include "Sim_Types.h"
#include "Linear_Algebra.h"
#include "Motor.h"
#include "Environment.h"
#include "Coordinate_Frames.h"
#include "Controllers.h"

using namespace std;

#define STANDARD_WIDTH 20
#define LOG_SIM(data) (log_sim << setw(STANDARD_WIDTH) << data)
#define LOG_MCU(data) (log_mcu << setw(STANDARD_WIDTH) << data)


#define BAR_SENS 1.0/40.96 // Pa/LSB
#define BAR_WINDOW_SIZE 16
#define BAR_TB 288.15 // Standard temperature at sea level (K)
#define BAR_G 9.8065 // Acceleration due to gravity (m/s^2)
#define BAR_LB -0.0065 // Standard temperature lapse rate (K/m)
#define BAR_PB 101325.0 // Standard static pressure at sea level (Pa)
#define BAR_R 8.31432 // Universal gas constant (J/(mol-K))S
#define BAR_M 0.0289644 // Molar mass of air (kg/mol)

#define MAG_WINDOW_SIZE 4

#define IMU_WINDOW_SIZE 8 // Size of FIR window
#define GYRO_WINDOW_SIZE 2
#define GYRO_SENS 500.0/32768.0
#define ACCEL_SENS 2.0/32768.0

float Height_Bar(uint32_t pressure_LSB);

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
        States MCU = {0};
        // Desired state of the drone, should mirror its counterpart in src/Flight_Controller
        States Reference = {0};
        // Model of BLDC motor and propeller
        Motor Motors[4]; // Back (CW), Left (CCW), Front (CW), Right (CCW)
        // Flag to enable guidance and control functions in MCU after calibration
        bool MCU_Cal_Flag = false;
        // Desired thrust by MCU
        float Desired_Thrust;
        // Desired moments by MCU
        float Desired_Moments[3];
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
            Differential_equation_momentum(Vec x_in);
};