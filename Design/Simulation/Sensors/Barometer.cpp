#include "Barometer.h"

void Barometer::Initialize(uint16_t odr, uint8_t watermark, Barometer_mode mode, uint8_t filter_setting){
    ODR = odr;
    Update_rate.MicroSeconds = 1000000/ODR;
    FIFO_watermark = watermark;
    Mode = mode;
    last_sample_time.Seconds = 0;
    last_sample_time.MicroSeconds = 0;

    Filter_Setting = filter_setting;
    if (Filter_Setting > 2){
        Filter_Setting = 0;
        passthrough_flag = true;
    }

    Gaussian_Bar.Initialize(noise_rms, 0.0);
    Bar_Filter.Initialize(static_cast<double>(ODR)/Low_Pass_Filter_BW[Filter_Setting], 0.0);
}

void Barometer::Sample(Environment &env, Sim_Time sim_t){
    
    if ((sim_t - last_sample_time) < Update_rate) return;
    last_sample_time = sim_t;
    // Get noise in hpa
    double noise_hpa = Gaussian_Bar.Get_val();
    // Add to true data in hPa
    double pressure_combined_hpa =(env.pressure/100.0) + noise_hpa;
    // Run through onboard low pass filter
    double filtered_pressure_hpa = Bar_Filter.Update(pressure_combined_hpa, Update_rate.Time_fp(), passthrough_flag);
    // Convert from hpa to LSB
    double pressure_LSB = filtered_pressure_hpa*sensitivity;
    // Don't worry about saturating as this sim will never be near that limit
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
