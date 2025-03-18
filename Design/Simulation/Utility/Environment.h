#pragma once

#include <vector>
#include <cstdlib>
#include <cmath>
#include "Linear_Algebra.hpp"
#include "Coordinate_Frames.h"

using namespace std;

#define PI 3.14159265358979311600
#define D2R PI/180.0
class Environment{
    private:
        // Longitude (rad), Latitude (rad), and altitude (m) of reference (initial) position
        Vec3 lla;
        // lla of current position
        Vec3 lla_current;
        // Drone start position in ecef coordinates (m) used as a reference for future conversions
        Vec3 P_ref_ecef;
        // Current drone position in ecef coordinates (m)
        Vec3 P_ecef;
        double
            a_c = 6378137.0, // Equator radius, a
            b_c = 6356752.314, // Polar radius, b
            e_c = sqrt((pow(a_c,2) - pow(b_c, 2))/pow(a_c,2)), // Earth eccentricity
            g_e_c = 9.780, // Gravity at the equator, m/s^2
            g_p_c = 9.832, // Gravity at the poles, m/s^2
            k_c = ((b_c*g_p_c) - (a_c*g_e_c))/(a_c*g_e_c); // Gravity constant
            // Assume an orthometric height for simulation purposes in (m)
        // From the LPS22H, there is a low pressure sensor noise of 0.65 Pa (65 hPa)
        const double bar_sens = 40.96;
        const double bar_max_noise = 0.65;
        const double bar_noise_sens = 32767.0/bar_max_noise;
        const double H_ortho = 50;
        const double R = 8.3144598;
        const double g_0 = 9.80665;
        const double M_0 = 0.0289644;
        const double h_b = 0.0;
        const double L_m = 0.0065;
        const double T_m = 288.15;
        const double P_b = 101325.0;
        const double p_c2 = (g_0*M_0)/(R*L_m);

        const double mag_inclination = 60.98*D2R;
        const double mag_declination = 2.76*D2R;
        // Field strength in mgauss
        const double mag_field_strength = 479; 
        Vec3 m_vec;
    public:
        // The below are TRUE variables, inaccessible directly by the MCU
        double
            gravity,
            height,
            pressure,
            ground_stiffness = 1000.0,
            ground_damping = ground_stiffness/10.0;
        // The below are SENSED variables, they are read by the flight controllers sensors
        uint32_t 
            pressure_LSB;
        // Methods
        Environment(double Longitude, double Latitude, double Altitude_MSL);
        void 
            Update(Vec3 Position_NED, Vec quaternion);
        uint32_t
            Get_pressure();
        array<int16_t, 3> 
            Get_magnetic_field();
};