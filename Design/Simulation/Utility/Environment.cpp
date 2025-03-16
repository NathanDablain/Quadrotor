#include "Environment.h"

Environment::Environment(double Longitude, double Latitude, double Altitude_MSL){
    // Store latitude, longitude, and altitude
    lla = {Latitude*D2R, Longitude*D2R, Altitude_MSL};
    // Update ecef reference position
    P_ref_ecef = lla2ecef(lla);
}

void Environment::Update(Vec3 Position_NED, Vec quaternion){
    // Update ecef position based off of current NED position and established reference ecef position in constructor
    P_ecef = NED2ecef(Position_NED, P_ref_ecef, lla);
    // Update lla position based off of updated ecef
    lla_current = ecef2lla(P_ecef);
    // Update gravity based off of latitude
    gravity = g_e_c*((1 + k_c*pow(sin(lla_current.data[0]), 2))/sqrt(1 - (pow(e_c , 2)*pow(sin(lla_current.data[0]), 2))));
    // Update pressure based off of altitude

}

uint32_t Environment::Get_pressure(){


    return pressure_LSB;
}