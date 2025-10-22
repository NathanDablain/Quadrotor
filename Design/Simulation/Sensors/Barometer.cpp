#include "Barometer.h"
#include "External_Interface.h"

void Barometer::Initialize(uint16_t odr, bool low_noise, uint8_t filter_setting){
    ODR = odr;
    Update_rate.MicroSeconds = 1000000/ODR;
    last_sample_time.Seconds = 0;
    last_sample_time.MicroSeconds = 0;

    // Low noise mode only works on ODRs <=75Hz
    switch(odr){
        case 1:
            low_noise_en = low_noise;
            ODR = odr;
            break;
        case 10:
            low_noise_en = low_noise;
            ODR = odr;
            break;
        case 25:
            low_noise_en = low_noise;
            ODR = odr;
            break;
        case 50:
            low_noise_en = low_noise;
            ODR = odr;
            break;
        case 75:
            low_noise_en = low_noise;
            ODR = odr;
            break;
        case 100:
            ODR = odr;
            break;
        case 200:
            ODR = odr;
            break;
        default:
            ODR = 1;
            break;
    }
    uint8_t filter_index2 = (low_noise_en)?(0):(1);
    if (filter_setting > 2){
        filter_setting = 0;
    }
    // Add 50% FOS to the datasheet value
    double noise_FOS = 1.5*noise_rms[filter_index2][filter_setting];

    Gaussian_Bar.Initialize(noise_FOS, 0.0);
    Bar_Filter.Initialize(static_cast<double>(ODR)/Low_Pass_Filter_BW[filter_setting], 0.0);
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
    e_bar_data[0] = (uint8_t)Pressure_Out_LSB;
    e_bar_data[1] = (uint8_t)(Pressure_Out_LSB>>8);
    e_bar_data[2] = (uint8_t)(Pressure_Out_LSB>>16);
}