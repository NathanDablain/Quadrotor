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
    static Gaussian Gaussian_Bar(noise_rms, 0.0);

    if ((sim_t - last_sample_time) < Update_rate) return;
    last_sample_time = sim_t;
    // Get noise in hpa
    double noise_hpa = Gaussian_Bar.Get_val();
    // Add to true data in Pa, convert to LSB
    double pressure_LSB = (env.pressure/100.0 + noise_hpa)*sensitivity;

    Pressure_Out_LSB = static_cast<uint32_t>(pressure_LSB);
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
