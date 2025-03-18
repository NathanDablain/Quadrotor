#include "Environment.h"

Environment::Environment(double Longitude, double Latitude, double Altitude_MSL){
    // Store latitude, longitude, and altitude
    lla = {Latitude*D2R, Longitude*D2R, Altitude_MSL};
    // Update ecef reference position
    P_ref_ecef = lla2ecef(lla);
    // Rotate perfect magnetic field with
    m_vec = {mag_field_strength, 0.0, 0.0};
}

void Environment::Update(Vec3 Position_NED, Vec quaternion){
    // Update ecef position based off of current NED position and established reference ecef position in constructor
    P_ecef = NED2ecef(Position_NED, P_ref_ecef, lla);
    // Update lla position based off of updated ecef
    lla_current = ecef2lla(P_ecef);
    // Update gravity based off of latitude
    gravity = g_e_c*((1 + k_c*pow(sin(lla_current.data[0]), 2))/sqrt(1 - (pow(e_c , 2)*pow(sin(lla_current.data[0]), 2))));
    // Update pressure based off of altitude
    double p_c1 = (L_m/T_m)*(H_ortho + lla_current.data[2] - h_b);
    pressure = P_b*pow(1.0 - p_c1, p_c2);
    // Update magnetic field based off of quaternion
}

uint32_t Environment::Get_pressure(){
    double P_hp = pressure*bar_sens;

    double random_noise = 2.0*rand()/bar_noise_sens;
    random_noise *= bar_sens;
    int16_t pressure_noise_LSB = (random_noise>(bar_max_noise*bar_sens))?(static_cast<int16_t>(random_noise-(bar_max_noise*bar_sens))):(static_cast<int16_t>(random_noise));
    pressure_LSB = static_cast<uint32_t>(P_hp) + pressure_noise_LSB;

    return pressure_LSB;
}

array<int16_t, 3> Environment::Get_magnetic_field(){

}