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
};