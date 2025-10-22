#include <stdint.h>
#include <string.h>
#include <math.h>
#include "Global_Variables_p32.h"
#include "External_Interface.h"
#include "Sim_Types.h"
#include "Barometer_p32.h"

static BAR_Data barometer = {0};
static BAR_Machine state;
static bool offset_initialized;

void Initialize_Bar(){
    memset(&barometer, 0, sizeof(barometer));
    offset_initialized = false;
    state = BAR_Standby;
}

void Run_Barometer_Machine(){
    const int32_t ODR_Hz = 75;
    const Time Sample_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Hz};
    
    switch (state){
        case BAR_Standby:
            e_bar_odr = 75;
            e_bar_lpf_setting = 0;
            e_bar_low_noise_setting = true;
            e_bar_settings_updated = true;
            state = BAR_Ready;
            break;
        case BAR_Fail:
            break;
        case BAR_Ready:
            if (Compare_And_Update(Current_Time(), Sample_Rate, &barometer.Last_Update)){
                state = BAR_Reading;
            }
            break;
        case BAR_Reading:
            Convert_Pressure();
            state = BAR_Ready;
            break;
    }

}

void Convert_Pressure(){
    const double sensitivity = 1.0/40.96; // Pa/LSB
    const double standard_temp = 288.15; // Standard temperature at sea level (K)
    const double gravity = 9.8065; // Acceleration due to gravity (m/s^2)
    const double lapse_rate = -0.0065; // Standard temperature lapse rate (K/m)
    const double standard_pressure = 101325.0; // Standard static pressure at sea level (Pa)
    const double gas_constant = 8.31432; // Universal gas constant (J/(mol-K))S
    const double molar_mass = 0.0289644; // Molar mass of air (kg/mol)
    
    const double c1 = standard_temp/lapse_rate;
    const double c2 = -(gas_constant*lapse_rate)/(gravity*molar_mass);
    
    barometer.pressure_LSB_bytes[1] = e_bar_data[0];
    barometer.pressure_LSB_bytes[2] = e_bar_data[1];
    barometer.pressure_LSB_bytes[3] = e_bar_data[2];

    uint32_t pressure_LSB = barometer.pressure_LSB_bytes[1] +
            (((uint16_t)barometer.pressure_LSB_bytes[2])<<8) + 
            (((uint32_t)barometer.pressure_LSB_bytes[3])<<16);
    barometer.pressure_pa = ((double)pressure_LSB)*sensitivity;
    barometer.height = c1*(pow(barometer.pressure_pa/standard_pressure, c2) - 1.0) - barometer.base_altitude;
    
    if (g_Flight_Controller_Status == System_Calibration && !offset_initialized){
        barometer.base_altitude = barometer.height;
        offset_initialized = true;
    }
}

double Barometer_Altitude(){
    return barometer.height;
}

double Barometer_Pressure(){
    return barometer.pressure_pa;
}