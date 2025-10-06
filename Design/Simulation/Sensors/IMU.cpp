#include "IMU.h"

using namespace std;

void IMU::Initialize(uint16_t gyro_odr, uint16_t accel_odr, uint16_t accel_watermark, uint8_t gyro_lpf_setting, uint8_t accel_lpf_setting){
    // Add some error from the ideal sampling rate
    ODR_Gyro = gyro_odr - gyro_odr/50; // Given in Hz
    ODR_Accel = accel_odr - accel_odr/50; // Given in Hz

    Update_Rate_Gyro.Seconds = 0;
    Update_Rate_Gyro.MicroSeconds = 1000000/ODR_Gyro;
    Update_Rate_Accel.Seconds = 0;
    Update_Rate_Accel.MicroSeconds = 1000000/ODR_Accel;
    last_sample_time_gyro.Seconds = 0;
    last_sample_time_gyro.MicroSeconds = 0;
    last_sample_time_accel.Seconds = 0;
    last_sample_time_accel.MicroSeconds = 0;
    time_last_walk_update.Seconds = 0;
    time_last_walk_update.MicroSeconds = 0;
    Random_walk_rate.Seconds = 1;
    Random_walk_rate.MicroSeconds = 0;
    gyro_random_walk_mdps = 0.0;

    gyro_bias.data[0] = 1.4 * 1000.0;
    gyro_bias.data[1] = -3.2 * 1000.0;
    gyro_bias.data[2] = 2.5 * 1000.0;

    // accel_bias.data[0] = 38.0; // 622
    // accel_bias.data[1] = -12.0; // -196
    // accel_bias.data[2] = 42.0; // 688

    FIFO_watermark = accel_watermark;
    // data sheet value plus 50% FOS
    accel_noise_rms = 1.5*0.09*sqrt(static_cast<double>(accel_odr));
    gyro_noise_rms = 1.5*5.0*sqrt(static_cast<double>(gyro_odr));

    Set_Filter_Settings(gyro_lpf_setting, accel_lpf_setting);

    accel_gaussian.Initialize(accel_noise_rms, 0.0);
    gyro_gaussian.Initialize(gyro_noise_rms, 0.0);
    accel_filter_x.Initialize(static_cast<double>(ODR_Accel)/Accel_filter_BW[Accel_filter_setting], 0.0);
    accel_filter_y.Initialize(static_cast<double>(ODR_Accel)/Accel_filter_BW[Accel_filter_setting], 0.0);
    accel_filter_z.Initialize(static_cast<double>(ODR_Accel)/Accel_filter_BW[Accel_filter_setting], 0.0);
    gyro_filter_x.Initialize(Gyro_Filter_BW, 0.0);
    gyro_filter_y.Initialize(Gyro_Filter_BW, 0.0);
    gyro_filter_z.Initialize(Gyro_Filter_BW, 0.0);
}

void IMU::Set_Filter_Settings(uint8_t gyro_lpf_setting, uint8_t accel_lpf_setting){
    Accel_filter_setting = accel_lpf_setting;
    if (Accel_filter_setting > 3){
        Accel_filter_setting = 0;
        Accel_passthrough_flag = true;
    }
    Gyro_filter_setting = gyro_lpf_setting;
    if (Gyro_filter_setting > 3){
        Gyro_filter_setting = 0;
        Gyro_passthrough_flag = true;
    }
    switch (ODR_Gyro){
        case 833:
            Gyro_Filter_BW = Gyro_ODR_833_BW[Gyro_filter_setting];
            break;
        case 1660:
            Gyro_Filter_BW = Gyro_ODR_1660_BW[Gyro_filter_setting];
            break;
        case 3330:
            Gyro_Filter_BW = Gyro_ODR_3330_BW[Gyro_filter_setting];
            break;
        case 6660:
            Gyro_Filter_BW = Gyro_ODR_6660_BW[Gyro_filter_setting];
            break;
        default:
            Gyro_Filter_BW = static_cast<double>(ODR_Gyro);
            Gyro_passthrough_flag = true;
            break;
    }
}

void IMU::Sample_Acc(Environment &env, Sim_Time &sim_t){
    if (sim_t - last_sample_time_accel < Update_Rate_Accel) return;
    last_sample_time_accel = sim_t;
    // The accelerometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = -Sensor z
    // 1. add noise in mg
    Vec3 accel_noise;
    accel_noise.data[0] = accel_gaussian.Get_val() + accel_bias.data[0];
    accel_noise.data[1] = accel_gaussian.Get_val() + accel_bias.data[0];
    accel_noise.data[2] = accel_gaussian.Get_val() + accel_bias.data[0];
    // 2. saturate
    Vec3 accel_output = {Saturate((env.a_measured.data[0]/env.gravity)*1000.0 + accel_noise.data[0], -accel_range, accel_range),
                         Saturate((env.a_measured.data[1]/env.gravity)*1000.0 + accel_noise.data[1], -accel_range, accel_range),
                         Saturate((env.a_measured.data[2]/env.gravity)*1000.0 + accel_noise.data[2], -accel_range, accel_range)};
    // 3. run through onboard low pass filter
    Vec3 accel_filtered_output = {accel_filter_x.Update(accel_output.data[0], Update_Rate_Accel.Time_fp(), Accel_passthrough_flag),
                                  accel_filter_y.Update(accel_output.data[1], Update_Rate_Accel.Time_fp(), Accel_passthrough_flag),
                                  accel_filter_z.Update(accel_output.data[2], Update_Rate_Accel.Time_fp(), Accel_passthrough_flag)};
    // 4. convert from mg to LSB
    Vec3 accel_LSB = accel_filtered_output*(1.0/accel_sens);
    acceleration_LSB[0] = static_cast<int16_t>(accel_LSB.data[0]);
    acceleration_LSB[1] = static_cast<int16_t>(-accel_LSB.data[1]);
    acceleration_LSB[2] = static_cast<int16_t>(-accel_LSB.data[2]);
    // 5. set data ready flag
    accel_drdy_flag = true;
}

void IMU::Sample_Gyr(Environment &env, Vec3 &w, Sim_Time &sim_t){
    if (sim_t - time_last_walk_update > Random_walk_rate){
        time_last_walk_update = sim_t;
        gyro_random_walk_mdps += gyro_random_walk_rate_mdps;
    }

    if (sim_t - last_sample_time_gyro < Update_Rate_Gyro) return;
    last_sample_time_gyro = sim_t;
    // The gyroscope sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = -Sensor z
    // 1. add noise and bias in mdps
    Vec3 gyro_noise;
    gyro_noise.data[0] = gyro_gaussian.Get_val() + gyro_bias.data[0] + gyro_random_walk_mdps;
    gyro_noise.data[1] = gyro_gaussian.Get_val() + gyro_bias.data[1] + gyro_random_walk_mdps;
    gyro_noise.data[2] = gyro_gaussian.Get_val() + gyro_bias.data[2] + gyro_random_walk_mdps;
    // 2. saturate
    Vec3 gyro_output= {Saturate(w.data[0]*R2D + (gyro_noise.data[0]/1000.0), -gyro_range, gyro_range),
                        Saturate(w.data[1]*R2D + (gyro_noise.data[1]/1000.0), -gyro_range, gyro_range),
                        Saturate(w.data[2]*R2D + (gyro_noise.data[2]/1000.0), -gyro_range, gyro_range)};
    // 3. run through onboard low pass filter
    Vec3 Filtered_gyro_output = {gyro_filter_x.Update(gyro_output.data[0], Update_Rate_Gyro.Time_fp(), Gyro_passthrough_flag),
                                 gyro_filter_y.Update(gyro_output.data[1], Update_Rate_Gyro.Time_fp(), Gyro_passthrough_flag),
                                 gyro_filter_z.Update(gyro_output.data[2], Update_Rate_Gyro.Time_fp(), Gyro_passthrough_flag)};
    // 4. convert from dps to LSB
    Vec3 gyro_LSB = Filtered_gyro_output*(1.0/gyro_sens_dps);
    angular_rate_LSB[0] = static_cast<int16_t>(gyro_LSB.data[0]);
    angular_rate_LSB[1] = static_cast<int16_t>(-gyro_LSB.data[1]);
    angular_rate_LSB[2] = static_cast<int16_t>(-gyro_LSB.data[2]);
    // 5. set data ready flag
    gyro_drdy_flag = true;
}

void IMU::Read_FIFO(std::array<int16_t, 3> *out){
    for (int8_t i=FIFO_index-1; i>=0; i--){
        *out = FIFO_buffer[i];
        out++;
    }
    FIFO_index = 0;
}