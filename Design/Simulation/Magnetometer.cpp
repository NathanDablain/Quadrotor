#include "Magnetometer.h"

void Magnetometer::Initialize(uint16_t odr){
    ODR = odr;
    Update_Rate.Seconds = 0;
    Update_Rate.MicroSeconds = 1000000/ODR;
    last_sample_time.Seconds = 0;
    last_sample_time.MicroSeconds = 0;
}

void Magnetometer::Sample(Environment &env, Sim_Time sim_t){
    if ((sim_t - last_sample_time) < Update_Rate) return;
    last_sample_time = sim_t;
    // The magnetometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor y
    // -> Body y = -Sensor x
    // -> Body z = Sensor z
    Vec3 mag_LSB_true = (env.m_vec_Body + env.mag_hard_iron)*mag_sens;
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
    drdy_flag = true;
}