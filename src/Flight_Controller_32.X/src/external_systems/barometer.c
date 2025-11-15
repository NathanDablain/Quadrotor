#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "global_variables.h"
#include "barometer.h"
#include "spi.h"
#include "pins.h"
#include "time.h"
#include "dma.h"
#include "lora.h"
#include "butterworth_filter.h"

static BAR_Data       barometer;
static BAR_Machine    state = BAR_Standby;
static BW_Filter_Data BW_Filter;
static bool           offset_initialized;

void Initialize_Barometer_Machine(){
    memset(&barometer, 0, sizeof(barometer));
    Initialize_BW_Filter(&BW_Filter, 1.0/416.0);
    BW_Filter.w_c = 5.0;
    barometer.BAR_Read_array[0] = (BAR_DATA_START | 0x80);
    barometer.drdy_flag = false;
    offset_initialized = false;
}

void Run_Barometer_Machine(){
    
    switch (state){
        case BAR_Standby:
            state = Initialize_Barometer();
            break;
        case BAR_Fail:
            break;
        case BAR_Ready:
            if (g_spi1_rdy_flag && g_barometer_sample_flag){
                g_barometer_sample_flag = false;
                Prepare_SPI1_For_DMA(&CS_BAR_PORT, CS_BAR_PIN,  barometer.BAR_Read_array, &barometer.drdy_flag);
                Set_DMA_01(&barometer.BAR_Read_array[1], &barometer.pressure_LSB_bytes[0], sizeof(barometer.pressure_LSB_bytes));
                state = BAR_Reading;
            }
            if (g_Flight_Controller_Status >= User_Calibration){
                // Run filter in user calibration mode to get filtered base altitude 
                if (g_barometer_filter_flag){
                    g_barometer_filter_flag = false;
                    BW_Filter.u = barometer.height_measurement;
                    barometer.height = Run_BW_Filter(&BW_Filter);
                    if (g_Flight_Controller_Status == System_Calibration && !offset_initialized){
                        barometer.Base_Pressure_LSB = Uplink_Pressure_LSB();
                        barometer.base_altitude = barometer.height;
                        BW_Filter.x[0] = 0.0;
                        offset_initialized = true;
                    }
                }
            }
            else{
                // Bypass filter when in standby mode
                barometer.height = barometer.height_measurement;
                BW_Filter.x[0] = barometer.height_measurement;
            }
            break;
        case BAR_Reading:
            if ( barometer.drdy_flag){
                if (g_Flight_Controller_Status > System_Calibration){
                    barometer.pressure_offset  = Uplink_Pressure_LSB() - barometer.Base_Pressure_LSB;
                }
                Convert_Pressure();
                barometer.drdy_flag = false;
                state = BAR_Ready;
            }
            break;
    }

}

BAR_Machine Initialize_Barometer(){
    uint8_t data_in[2] = {BAR_CTRL_REG2, BAR_RST};
    uint8_t data_out[2] = {0};
    
    SPI_transfer(&CS_BAR_PORT, CS_BAR_PIN, data_in, data_out, sizeof(data_in));
    // Give device time to reboot before setting parameters
	Delay(10000000);
    
    data_in[0] = (BAR_WHO_AM_I | 0x80);
    SPI_transfer(&CS_BAR_PORT, CS_BAR_PIN, data_in, data_out, sizeof(data_in));
	if (data_out[1] != BAR_ID) return BAR_Fail;
    
    // Set 75Hz update rate, ODR/2 bandwidth, and block data update
    data_in[0] = BAR_CTRL_REG1;
    data_in[1] = BAR_ODR_75 | BAR_BDU | BAR_LPF | BAR_LPF_CFG;
    SPI_transfer(&CS_BAR_PORT, CS_BAR_PIN, data_in, data_out, sizeof(data_in));
    
    // Set low noise mode
    data_in[0] = BAR_CTRL_REG2;
    data_in[1] = BAR_ADD_INC | BAR_LOW_NOISE;
    SPI_transfer(&CS_BAR_PORT, CS_BAR_PIN, data_in, data_out, sizeof(data_in));
    
    return BAR_Ready;
}

void Convert_Pressure(){
    const double sensitivity       = 1.0/40.96; // Pa/LSB
    const double standard_temp     = 288.15; // Standard temperature at sea level (K)
    const double gravity           = 9.8065; // Acceleration due to gravity (m/s^2)
    const double lapse_rate        = -0.0065; // Standard temperature lapse rate (K/m)
    const double standard_pressure = 101325.0; // Standard static pressure at sea level (Pa)
    const double gas_constant      = 8.31432; // Universal gas constant (J/(mol-K))S
    const double molar_mass        = 0.0289644; // Molar mass of air (kg/mol)
    
    const double c1 = standard_temp/lapse_rate;
    const double c2 = -(gas_constant*lapse_rate)/(gravity*molar_mass);
    // Will assume it is unreasonable to be more than ~1300 meters above sea level
    const uint32_t reasonable_pressure_LSB = standard_pressure * 41;
    const uint32_t reasonable_threshold_LSB = 15000 * 41;
    // seven digits in pressure_LSB
    uint32_t pressure_LSB = barometer.pressure_LSB_bytes[1] +
            (((uint16_t)barometer.pressure_LSB_bytes[2])<<8) + 
            (((uint32_t)barometer.pressure_LSB_bytes[3])<<16);
    
    if (abs(pressure_LSB - reasonable_pressure_LSB) > reasonable_threshold_LSB){
        return;
    }
    pressure_LSB -= barometer.pressure_offset;
    barometer.pressure_pa = ((double)pressure_LSB)*sensitivity;
    barometer.height_measurement = c1*(pow(barometer.pressure_pa/standard_pressure, c2) - 1.0) - barometer.base_altitude;

}

double Barometer_Altitude(){
    return barometer.height;
}

double Barometer_Pressure(){
    return barometer.pressure_pa;
}

uint32_t Barometer_Pressure_LSB(){
    return barometer.pressure_LSB;
}