#include "Environment.h"

Environment::Environment(double Longitude, double Latitude, double Altitude_MSL, Sim_Time Sim_dt){
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
    sim_dt = Sim_dt;
}

void Environment::Update(Vec3 &Position_NED, Vec &quaternion, Vec3 &v){
    // Update ecef position based off of current NED position and established reference ecef position in constructor
    P_ecef = NED2ecef(Position_NED, P_ref_ecef, lla);
    // Update lla position based off of updated ecef
    lla_current = ecef2lla(P_ecef);
    // Update gravity based off of latitude
    gravity = g_e_c*((1 + k_c*pow(sin(lla_current.data[0]), 2))/sqrt(1 - (pow(e_c , 2)*pow(sin(lla_current.data[0]), 2))));
    // Update pressure based off of altitude
    double p_c1 = (L_b/T_b)*(H_ortho - Position_NED.data[2] - h_b);
    pressure = P_b*pow(1.0 + p_c1, p_c2);
    // Update magnetic field based off of quaternion
    m_vec_Body = NED2Body(m_vec_NED, quaternion);
    // Update linear acceleration
    static Vec3 v_last = {0.0, 0.0, 0.0};
    dv_dt = dv_dt*accel_c1 + ((v-v_last)/sim_dt.Time_fp())*accel_c2;
    v_last = v;
}

double Saturate(double value_in, double low_limit, double high_limit){
    double value_out = (value_in > high_limit)?high_limit:value_in;
    if (value_out < low_limit){
        value_out = low_limit;
    }
    return value_out;
}