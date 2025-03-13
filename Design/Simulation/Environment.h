#pragma once

#include <vector>
#include <cstdlib>
#include <cmath>

using namespace std;

#define PI 3.14159265358979311600
#define D2R PI/180.0
class Environment{
    private:
        double 
            lla[3],
            a_c = 6378137.0, // Equator radius, a
            b_c = 6356752.314, // Polar radius, b
            e_c = sqrt((pow(a_c,2) - pow(b_c, 2))/pow(a_c,2)), // Earth eccentricity
            g_e_c = 9.780, // Gravity at the equator, m/s^2
            g_p_c = 9.832, // Gravity at the poles, m/s^2
            k_c = ((b_c*g_p_c) - (a_c*g_e_c))/(a_c*g_e_c); // Gravity constant
    public:
        double
            gravity,
            height,
            pressure;
        Environment(double Longitude, double Latitude, double Altitude_MSL);
        void 
            Apply_forces_moments(vector<double> &Forces_NED, vector<double> &Moments_NED);
};