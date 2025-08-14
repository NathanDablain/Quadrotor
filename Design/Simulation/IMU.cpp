#include "IMU.h"

using namespace std;

void IMU::Initialize(uint16_t gyro_odr, uint16_t accel_odr, uint16_t accel_watermark){
    ODR_Gyro = gyro_odr; // Given in Hz
    ODR_Accel = accel_odr; // Given in Hz

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

    accel_bias.data[0] = 38.0;
    accel_bias.data[1] = -12.0;
    accel_bias.data[2] = 42.0;

    FIFO_watermark = accel_watermark;
    accel_noise_rms = 0.09*sqrt(static_cast<double>(accel_odr));
    gyro_noise_rms = 5.0*sqrt(static_cast<double>(gyro_odr));
}

void IMU::Sample_Acc(Environment &env, Vec &quaternion, Sim_Time &sim_t){
    static Gaussian accel_gaussian(accel_noise_rms, 0.0);

    if (sim_t - last_sample_time_accel < Update_Rate_Accel) return;
    last_sample_time_accel = sim_t;
    // The accelerometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = Sensor z
    Vec3 g_vec_NED = {0.0, 0.0, env.gravity};
    Vec3 g_vec_Body = NED2Body(g_vec_NED, quaternion);
    Vec3 v_dot = (g_vec_Body*1000.0);// + env.dv_dt)*1000.0;
    // 1. add noise in mg
    Vec3 accel_noise;
    accel_noise.data[0] = accel_gaussian.Get_val() + accel_bias.data[0];
    accel_noise.data[1] = accel_gaussian.Get_val() + accel_bias.data[0];
    accel_noise.data[2] = accel_gaussian.Get_val() + accel_bias.data[0];
    // 2. saturate
    Vec3 accel_output = {Saturate((v_dot.data[0]/env.gravity) + accel_noise.data[0], -accel_range, accel_range),
                         Saturate((v_dot.data[1]/env.gravity) + accel_noise.data[1], -accel_range, accel_range),
                         Saturate((v_dot.data[2]/env.gravity) + accel_noise.data[2], -accel_range, accel_range)};
    // 3. convert to LSB
    Vec3 accel_LSB = accel_output*(1.0/accel_sens);
    acceleration_LSB[0] = static_cast<int16_t>(accel_LSB.data[0]);
    acceleration_LSB[1] = static_cast<int16_t>(-accel_LSB.data[1]);
    acceleration_LSB[2] = static_cast<int16_t>(accel_LSB.data[2]);

    accel_drdy_flag = true;
}

void IMU::Sample_Gyr(Environment &env, Vec3 &w, Sim_Time &sim_t){
    static Gaussian gyro_gaussian(gyro_noise_rms, 0.0);

    if (sim_t - time_last_walk_update > Random_walk_rate){
        time_last_walk_update = sim_t;
        gyro_random_walk_mdps += gyro_random_walk_rate_mdps;
    }

    if (sim_t - last_sample_time_gyro < Update_Rate_Gyro) return;
    last_sample_time_gyro = sim_t;
    // The gyroscope sensor axis corresponds to the body axis in the following way
    // -> Body x = -Sensor x
    // -> Body y = Sensor y
    // -> Body z = -Sensor z
    // 1. add noise and bias in mdps, convert to dps
    Vec3 gyro_noise;
    gyro_noise.data[0] = gyro_gaussian.Get_val() + gyro_bias.data[0] + gyro_random_walk_mdps;
    gyro_noise.data[1] = gyro_gaussian.Get_val() + gyro_bias.data[1] + gyro_random_walk_mdps;
    gyro_noise.data[2] = gyro_gaussian.Get_val() + gyro_bias.data[2] + gyro_random_walk_mdps;
    // 2. saturate
    Vec3 gyro_output= {Saturate(w.data[0]*R2D + (gyro_noise.data[0]/1000.0), -gyro_range, gyro_range),
                        Saturate(w.data[1]*R2D + (gyro_noise.data[1]/1000.0), -gyro_range, gyro_range),
                        Saturate(w.data[2]*R2D + (gyro_noise.data[2]/1000.0), -gyro_range, gyro_range)};
    // 3. convert to LSB
    Vec3 gyro_LSB = gyro_output*(1.0/gyro_sens_dps);
    angular_rate_LSB[0] = static_cast<int16_t>(-gyro_LSB.data[0]);
    angular_rate_LSB[1] = static_cast<int16_t>(gyro_LSB.data[1]);
    angular_rate_LSB[2] = static_cast<int16_t>(-gyro_LSB.data[2]);
    // 4. set drdy flag
    gyro_drdy_flag = true;
}

void IMU::Read_FIFO(std::array<int16_t, 3> *out){
    for (int8_t i=FIFO_index-1; i>=0; i--){
        *out = FIFO_buffer[i];
        out++;
    }
    FIFO_index = 0;
}