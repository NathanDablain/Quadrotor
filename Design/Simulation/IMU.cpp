#include "IMU.h"

using namespace std;

void IMU::Initialize(uint16_t gyro_odr, uint16_t accel_odr, uint16_t accel_watermark){
    ODR_Gyro = gyro_odr;
    ODR_Accel = accel_odr;
    Update_Rate_Gyro.Seconds = 0;
    Update_Rate_Gyro.MicroSeconds = 1000000/ODR_Gyro;
    Update_Rate_Accel.Seconds = 0;
    Update_Rate_Accel.MicroSeconds = 1000000/ODR_Accel;
    last_sample_time_gyro.Seconds = 0;
    last_sample_time_gyro.MicroSeconds = 0;
    last_sample_time_accel.Seconds = 0;
    last_sample_time_accel.MicroSeconds = 0;
    FIFO_watermark = accel_watermark;
    accel_max_noise = 10.0*(sqrt(static_cast<double>(ODR_Accel)/2.0)*90.0)/1e6;
    gyro_max_noise = 10.0*(sqrt(static_cast<double>(ODR_Gyro)/2.0)*5.0/1000.0)*D2R;
}

void IMU::Sample_Acc(Environment &env, Vec &quaternion, Sim_Time &sim_t){
    if (sim_t - last_sample_time_accel < Update_Rate_Accel) return;
    last_sample_time_accel = sim_t;
    // The accelerometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = Sensor z
    Vec3 g_vec_NED = {0.0, 0.0, env.gravity};
    Vec3 g_vec_Body = NED2Body(g_vec_NED, quaternion);
    Vec3 v_dot = g_vec_Body + env.dv_dt;
    // 1. add noise
    Vec3 accel_noise;
    for (uint8_t i = 0; i < 3; i++){
        accel_noise.data[i] = 2.0*rand()*accel_max_noise/32767.0;
        if (accel_noise.data[i] > accel_max_noise){
            accel_noise.data[i] -= accel_max_noise;
        }
    }
    // 2. saturate
    Vec3 accel_output = {Saturate((v_dot.data[0]/env.gravity) + accel_noise.data[0], -accel_range, accel_range),
                         Saturate((v_dot.data[1]/env.gravity) + accel_noise.data[1], -accel_range, accel_range),
                         Saturate((v_dot.data[2]/env.gravity) + accel_noise.data[2], -accel_range, accel_range)};
    // 3. convert to LSB
    Vec3 accel_LSB = accel_output*(1.0/accel_sens);
    acceleration_LSB[0] = static_cast<int16_t>(accel_LSB.data[0]);
    acceleration_LSB[1] = static_cast<int16_t>(-accel_LSB.data[1]);
    acceleration_LSB[2] = static_cast<int16_t>(accel_LSB.data[2]);
    // 4. log to FIFO
    if (FIFO_index < FIFO_watermark){
        FIFO_buffer[FIFO_index++] = acceleration_LSB;
    }
}

void IMU::Sample_Gyr(Environment &env, Vec3 &w, Sim_Time &sim_t){
    if (sim_t - last_sample_time_gyro < Update_Rate_Gyro) return;
    last_sample_time_gyro = sim_t;
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