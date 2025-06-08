#pragma once

#include <vector>
#include <cstdlib>
#include <cmath>
#include "Linear_Algebra.h"
#include "Coordinate_Frames.h"
#include "../Sim_Types.h"
#include "Sim_Time.h"

using namespace std;

double Saturate(double value_in, double low_limit, double high_limit);
class Environment{
    private:
        // Simulation time step
        Sim_Time sim_dt;
        // Longitude (rad), Latitude (rad), and altitude (m) of reference (initial) position
        Vec3 lla;
        // lla of current position
        Vec3 lla_current;
        // Drone start position in ecef coordinates (m) used as a reference for future conversions
        Vec3 P_ref_ecef;
        // Current drone position in ecef coordinates (m)
        Vec3 P_ecef;
        // Equator radius, a
        const double a_c = 6378137.0;
        // Polar radius, b
        const double b_c = 6356752.314;
        // Earth eccentricity
        const double e_c = sqrt((pow(a_c,2) - pow(b_c, 2))/pow(a_c,2)); 
        // Gravity at the equator, m/s^2
        const double g_e_c = 9.780;
        // Gravity at the poles, m/s^2
        const double g_p_c = 9.832;
        // Gravity constant
        const double k_c = ((b_c*g_p_c) - (a_c*g_e_c))/(a_c*g_e_c); 
        // Assume an orthometric height for simulation purposes in (m)
        const double H_ortho = 50;
        const double R = 8.3144598;
        const double g_0 = 9.80665;
        const double M_0 = 0.0289644;
        const double h_b = 0.0;
        const double L_b = -0.0065;
        const double T_b = 288.15;
        const double P_b = 101325.0;
        const double p_c2 = -(g_0*M_0)/(R*L_b);
        // Local magnetic field inclination
        const double mag_inc = 60.98*D2R;
        // Local magnetic field declination
        const double mag_dec = 2.76*D2R;
        // Field strength in mgauss
        const double mag_field_strength = 478.885;
        // Rotation matrix from true north to local magnetic field
        Mat3 R_mag;
        Vec3 m_vec_NED;
        //---------------------------------------//
        // IIR constants for converting linear velocity to linear acceleration
        const double accel_c1 = 0.995;
        const double accel_c2 = 1.0-accel_c1;
        //---------------------------------------//
    public:
        // Vector for holding linear acceleration
        Vec3 dv_dt = {0.0, 0.0, 0.0};
        Vec3 m_vec_Body;
        // Constant offsets in the magnetometer readings due to the local environment in (mgauss) 
        Vec3 mag_hard_iron = {0}; //{100, -175, 200};
        // The below are TRUE variables, inaccessible directly by the MCU
        double gravity;
        double height;
        double pressure;
        double ground_stiffness = 1000.0;
        double ground_damping = ground_stiffness/10.0;
        // Methods
        Environment(double Longitude, double Latitude, double Altitude_MSL, Sim_Time sim_dt);
        void 
            Update(Vec3 &Position_NED, Vec &quaternion, Vec3 &v);
};