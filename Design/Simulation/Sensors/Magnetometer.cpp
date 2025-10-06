#include "Magnetometer.h"

void Magnetometer::Initialize(uint16_t odr, uint8_t lpf_setting){
    ODR = odr;
    Update_Rate.Seconds = 0;
    Update_Rate.MicroSeconds = 1000000/ODR;
    last_sample_time.Seconds = 0;
    last_sample_time.MicroSeconds = 0;

    filter_setting = lpf_setting;
    if (filter_setting > 1){
        filter_setting = 0;
        passthrough_flag = true;
    }

    Gaussian_mag.Initialize(noise_rms, 0.0);
    Mag_Filter_x.Initialize(static_cast<double>(ODR)/Low_Pass_Filter_BW[filter_setting], 0.0);
    Mag_Filter_y.Initialize(static_cast<double>(ODR)/Low_Pass_Filter_BW[filter_setting], 0.0);
    Mag_Filter_z.Initialize(static_cast<double>(ODR)/Low_Pass_Filter_BW[filter_setting], 0.0);
}

void Magnetometer::Sample(Environment &env, Sim_Time sim_t){

    if ((sim_t - last_sample_time) < Update_Rate) return;
    last_sample_time = sim_t;
    // The magnetometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor y
    // -> Body y = -Sensor x
    // -> Body z = Sensor z
    // Get noise in mgauss
    Vec3 mag_noise = {Gaussian_mag.Get_val(), Gaussian_mag.Get_val(), Gaussian_mag.Get_val()};
    // Add to truth and hard iron offset
    Vec3 mag_output = env.m_vec_Body + mag_noise + env.mag_hard_iron;
    // Run through low pass filter
    Vec3 filtered_mag_output = {Mag_Filter_x.Update(mag_output.data[0], Update_Rate.Time_fp(), passthrough_flag),
                                Mag_Filter_y.Update(mag_output.data[1], Update_Rate.Time_fp(), passthrough_flag),
                                Mag_Filter_z.Update(mag_output.data[2], Update_Rate.Time_fp(), passthrough_flag)};
    // Convert to LSB
    Vec3 mag_LSB = filtered_mag_output*(1.0/mag_sens);

    magnetic_field_LSB[0] = -static_cast<int16_t>(mag_LSB.data[1]);
    magnetic_field_LSB[1] = static_cast<int16_t>(mag_LSB.data[0]);
    magnetic_field_LSB[2] = static_cast<int16_t>(mag_LSB.data[2]);

    drdy_flag = true;
}