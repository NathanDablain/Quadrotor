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

void Environment::Update(Vec3 &Position_NED, Vec &quaternion){
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
    // The magnetometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor y
    // -> Body y = -Sensor x
    // -> Body z = Sensor z
    Vec3 mag_LSB_true = (m_vec_Body + mag_hard_iron)*mag_sens;
    double temp = mag_LSB_true.data[0];
    mag_LSB_true.data[0] = mag_LSB_true.data[1];
    mag_LSB_true.data[1] = -temp;

    array<int16_t, 3> mag_noise_LSB;
    for (uint8_t i = 0; i < 3; i++){
        double random_noise = 2.0*rand()*mag_noise_sens;
        random_noise *= mag_sens;
        mag_noise_LSB[i] = (random_noise>(mag_max_noise*mag_sens))?(static_cast<int16_t>(random_noise-(mag_max_noise*mag_sens))):(static_cast<int16_t>(random_noise));
        magnetic_field_LSB[i] = static_cast<int16_t>(mag_LSB_true.data[i]) + mag_noise_LSB[i];
    }

    return magnetic_field_LSB;
}

array<int16_t, 3> Environment::Get_angular_rate(Vec3 &w){
    // The gyroscope sensor axis corresponds to the body axis in the following way
    // -> Body x = -Sensor x
    // -> Body y = Sensor y
    // -> Body z = -Sensor z
    // 1. add noise
    Vec3 gyro_noise;
    for (uint8_t i = 0; i < 3; i++){
        gyro_noise.data[i] = 2.0*rand()*gyro_max_noise/32767.0;
        if (gyro_noise.data[i] > gyro_max_noise){
            gyro_noise.data[i] -= gyro_max_noise;
        }
    }
    // 2. saturate
    Vec3 gyro_output= {Saturate(w.data[0] + gyro_noise.data[0], -gyro_range, gyro_range),
                        Saturate(w.data[1] + gyro_noise.data[1], -gyro_range, gyro_range),
                        Saturate(w.data[2] + gyro_noise.data[2], -gyro_range, gyro_range)};
    // 3. convert to LSB
    Vec3 gyro_LSB = gyro_output*(1.0/gyro_sens);
    array<int16_t, 3> gyro_output_LSB;
    gyro_output_LSB[0] = static_cast<int16_t>(-gyro_LSB.data[0]);
    gyro_output_LSB[1] = static_cast<int16_t>(gyro_LSB.data[1]);
    gyro_output_LSB[2] = static_cast<int16_t>(-gyro_LSB.data[2]);

    return gyro_output_LSB;
}

array<int16_t, 3> Environment::Get_acceleration(Vec3 &v, double d_t, Vec &quaternion){
    // The accelerometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = Sensor z
    static Vec3 v_last;
    static Vec3 v_dot_last;
    const double c1 = 0.5;
    const double c2 = 1.0-c1;
    Vec3 g_vec_NED = {0.0, 0.0, gravity};
    Vec3 v_dot = v_dot_last*c1 + (((v-(v_last))/d_t) + NED2Body(g_vec_NED, quaternion))*c2;
    // 1. add noise
    Vec3 accel_noise;
    for (uint8_t i = 0; i < 3; i++){
        accel_noise.data[i] = 2.0*rand()*accel_max_noise/32767.0;
        if (accel_noise.data[i] > accel_max_noise){
            accel_noise.data[i] -= accel_max_noise;
        }
    }
    // 2. saturate
    Vec3 accel_output = {Saturate((v_dot.data[0]/gravity) + accel_noise.data[0], -accel_range, accel_range),
                         Saturate((v_dot.data[1]/gravity) + accel_noise.data[1], -accel_range, accel_range),
                         Saturate((v_dot.data[2]/gravity) + accel_noise.data[2], -accel_range, accel_range)};
    // 3. convert to LSB
    Vec3 accel_LSB = accel_output*(1.0/accel_sens);
    array<int16_t, 3> accel_output_LSB;
    accel_output_LSB[0] = static_cast<int16_t>(accel_LSB.data[0]);
    accel_output_LSB[1] = static_cast<int16_t>(-accel_LSB.data[1]);
    accel_output_LSB[2] = static_cast<int16_t>(accel_LSB.data[2]);

    v_last = v;
    v_dot_last = v_dot;

    return accel_output_LSB;
}

double Saturate(double value_in, double low_limit, double high_limit){
    double value_out = (value_in > high_limit)?high_limit:value_in;
    if (value_out < low_limit){
        value_out = low_limit;
    }
    return value_out;
}