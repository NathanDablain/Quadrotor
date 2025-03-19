#include "Environment.h"

Environment::Environment(double Longitude, double Latitude, double Altitude_MSL){
    // Store latitude, longitude, and altitude
    lla = {Latitude*D2R, Longitude*D2R, Altitude_MSL};
    // Update ecef reference position
    P_ref_ecef = lla2ecef(lla);
    // Rotate true north to local NED frame magnetic north
    Vec3 m_vec_true = {mag_field_strength, 0.0, 0.0};
    R_mag.data[0] = {cos(mag_dec)*cos(mag_inc), -sin(mag_dec), -cos(mag_dec)*sin(mag_inc)};
    R_mag.data[1] = {cos(mag_inc)*sin(mag_dec),  cos(mag_dec), -sin(mag_dec)*sin(mag_inc)};
    R_mag.data[2] = {             sin(mag_inc),           0.0,               cos(mag_inc)};
    m_vec_NED = R_mag*m_vec_true; 
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
    m_vec_Body = NED2Body(m_vec_NED, quaternion);
}

uint32_t Environment::Get_pressure(){
    double pressure_LSB_true = pressure*bar_sens;

    double random_noise = 2.0*rand()*bar_noise_sens;
    random_noise *= bar_sens;
    int16_t pressure_noise_LSB = (random_noise>(bar_max_noise*bar_sens))?(static_cast<int16_t>(random_noise-(bar_max_noise*bar_sens))):(static_cast<int16_t>(random_noise));
    pressure_LSB = static_cast<uint32_t>(pressure_LSB_true) + pressure_noise_LSB;

    return pressure_LSB;
}

array<int16_t, 3> Environment::Get_magnetic_field(){
    Vec3 mag_LSB_true = (m_vec_Body + mag_hard_iron)*mag_sens;

    array<int16_t, 3> mag_noise_LSB;
    for (uint8_t i = 0; i < 3; i++){
        double random_noise = 2.0*rand()*mag_noise_sens;
        random_noise *= mag_sens;
        mag_noise_LSB[i] = (random_noise>(mag_max_noise*mag_sens))?(static_cast<int16_t>(random_noise-(mag_max_noise*mag_sens))):(static_cast<int16_t>(random_noise));
        magnetic_field_LSB[i] = static_cast<int16_t>(mag_LSB_true.data[i]) + mag_noise_LSB[i];
    }

    return magnetic_field_LSB;
}