#pragma once
#include <vector>
#include <cstdlib>

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
        // Actual mass moments of drone in (kg-m^2)
        double Ixx = 0.00149;
        double Iyy = 0.00262;
        double Izz = 0.00149;
        // Step time for simulation in (s)
        double sim_dt;
        // Final time of simulation in (s)
        double sim_tf;
        // Inertia matrix
        vector<vector<double>> inertia = {{Ixx, 0.0, 0.0},{0.0, Iyy, 0.0},{0.0, 0.0, Izz}};
        // Forces applied to drone at each time step in North-East-Down coordinate frame (N)
        vector<double> Forces_NED;
        // Moments applied to drone at each time step in North-East-Down coordinate frame (N-m)
        vector<double> Moments_NED;
        // Current state of the drone according to the MCU, should mirror the performance in src/Flight_Controller
        States MCU;
        // Desired state of the drone, should mirror its counterpart in src/Flight_Controller
        States Reference;
    public:
        Quadrotor(double Sim_dt, double Sim_tf);
        void
            Update_drone_states(),
            Run_sim();
};