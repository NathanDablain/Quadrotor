#include "Magnetometer.h"

void Magnetometer::Initialize(uint16_t odr){
    ODR = odr;
    Update_Rate.Seconds = 0;
    Update_Rate.MicroSeconds = 1000000/ODR;
    last_sample_time.Seconds = 0;
    last_sample_time.MicroSeconds = 0;
}

void Magnetometer::Sample(Environment &env, Sim_Time sim_t){
    static Gaussian Gaussian_mag(noise_rms, 0.0);
    static Low_Pass_Filter Mag_Filter_x(static_cast<double>(ODR)/Low_Pass_Filter_Setting, 0.0);
    static Low_Pass_Filter Mag_Filter_y(static_cast<double>(ODR)/Low_Pass_Filter_Setting, 0.0);
    static Low_Pass_Filter Mag_Filter_z(static_cast<double>(ODR)/Low_Pass_Filter_Setting, 0.0);

    if ((sim_t - last_sample_time) < Update_Rate) return;
    last_sample_time = sim_t;
    // The magnetometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor y
    // -> Body y = -Sensor x
    // -> Body z = Sensor z
    // hard irons applied are [-115, -67, 133]
    // Get noise in mgauss
    Vec3 mag_noise = {Gaussian_mag.Get_val(), Gaussian_mag.Get_val(), Gaussian_mag.Get_val()};
    // Add to truth and hard iron offset, convert to LSB
    Vec3 filtered_mag_output = {Mag_Filter_x.Update(env.m_vec_Body.data[0], Update_Rate.Time_fp()),
                                Mag_Filter_y.Update(env.m_vec_Body.data[1], Update_Rate.Time_fp()),
                                Mag_Filter_z.Update(env.m_vec_Body.data[2], Update_Rate.Time_fp())};
    Vec3 mag_LSB = (filtered_mag_output + env.mag_hard_iron + mag_noise)*(1.0/mag_sens);

    magnetic_field_LSB[0] = -static_cast<int16_t>(mag_LSB.data[1]);
    magnetic_field_LSB[1] = static_cast<int16_t>(mag_LSB.data[0]);
    magnetic_field_LSB[2] = static_cast<int16_t>(mag_LSB.data[2]);

    drdy_flag = true;
}