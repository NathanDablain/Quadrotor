#include "Environment.h"

Environment::Environment(double Longitude, double Latitude, double Altitude_MSL){
    // Store longitude, latitude, and altitude
    lla[0] = Longitude*D2R; lla[1] = Latitude*D2R; lla[2] = Altitude_MSL;
    // Update gravity based off of latitude
    gravity = g_e_c*((1 + k_c*pow(sin(lla[1]), 2))/sqrt(1 - (pow(e_c , 2)*pow(sin(lla[1]), 2))));
}