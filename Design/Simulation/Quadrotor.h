#pragma once

#include <vector>
#include <cstdlib>
#include <cstdint>
#include "Linear_Algebra.hpp"
#include "Motor.h"

using namespace std;

struct States{
    float w[3];
    float Euler[3];
    float g_vec[3];
    float m_vec[3];
    float pressure_altitude;
    float Position_NED[3];
    int32_t Longitude;
    int32_t Latitude;
    float Position_ECEF[3];
};

class Quadrotor{
    private:
        // Actual mass of drone in (kg)
        double mass = 0.441;
        // Distance from drone center of gravity to motor thrust vector in (m)
        double length =  0.1;
        // Actual mass moments of drone in (kg-m^2)
        double Ixx = 0.00149;
        double Iyy = 0.00262;
        double Izz = 0.00149;
        // Step time for simulation in (s)
        double sim_dt;
        // Final time of simulation in (s)
        double sim_tf;
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
            Run_sim();
        Vec
            Differential_equation_momentum(Vec &x_in);
};