#include "Barometer.h"

void Barometer::Initialize(uint16_t odr, uint8_t watermark, Barometer_mode mode){
    ODR = odr;
    Update_rate.MicroSeconds = 1000000/ODR;
    FIFO_watermark = watermark;
    Mode = mode;
    last_sample_time.Seconds = 0;
    last_sample_time.MicroSeconds = 0;
}

void Barometer::Sample(Environment &env, Sim_Time sim_t){
    if ((sim_t - last_sample_time) < Update_rate) return;
    last_sample_time = sim_t;

    double pressure_LSB_true = env.pressure*sensitivity;

    double random_noise = 2.0*rand()*noise_sensitivity;
    random_noise *= sensitivity;
    int16_t pressure_noise_LSB = (random_noise>(max_noise*sensitivity))?(static_cast<int16_t>(random_noise-(max_noise*sensitivity))):(static_cast<int16_t>(random_noise));
    Pressure_Out_LSB = static_cast<uint32_t>(pressure_LSB_true) + pressure_noise_LSB;
    drdy_flag = true;
    if ((Mode == Bar_Mode_FIFO)&&(FIFO_index < FIFO_watermark)){
        FIFO_buffer[FIFO_index++] = Pressure_Out_LSB;
    }
}

void Barometer::Read_FIFO(uint32_t *out){
    for (int8_t i=FIFO_index-1; i>=0; i--){
        *out = FIFO_buffer[i];
        out++;
    }
    FIFO_index = 0;
}
