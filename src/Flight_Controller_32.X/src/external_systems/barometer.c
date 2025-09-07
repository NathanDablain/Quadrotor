#include <xc.h>
#include <stdint.h>
#include <math.h>
#include "global_variables.h"
#include "barometer.h"
#include "spi.h"
#include "pins.h"
#include "time.h"
#include "dma.h"

static BAR_Data barometer = {0};
static BAR_Machine state = BAR_Standby;

void Run_Barometer_Machine(){
    const int32_t ODR_Hz = 75;
    const Time Sample_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Hz};
    static uint8_t BAR_Read_array[4] = {(BAR_DATA_START | 0x80), 0, 0, 0};
    static Time Last_Update = {0};
    static bool BAR_data_ready_flag = false;
    
    switch (state){
        case BAR_Standby:
            state = Initialize_Barometer();
            break;
        case BAR_Fail:
            break;
        case BAR_Ready:
            if ((g_spi1_rdy_flag) && (Compare_And_Update(Current_Time(), Sample_Rate, &Last_Update))){
                Prepare_SPI1_For_DMA(&CS_BAR_PORT, CS_BAR_PIN, BAR_Read_array, &BAR_data_ready_flag);
                Set_DMA_01(&BAR_Read_array[1], &barometer.pressure_LSB_bytes[0], sizeof(barometer.pressure_LSB_bytes));
                state = BAR_Reading;
            }
            break;
        case BAR_Reading:
            if (BAR_data_ready_flag){
                Convert_Pressure();
                BAR_data_ready_flag = false;
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
    
    // Set 75Hz update rate, ODR/20 bandwidth, and block data update
    data_in[0] = BAR_CTRL_REG1;
    data_in[1] = BAR_ODR_75 | BAR_LPF | BAR_LPF_CFG | BAR_BDU;
    SPI_transfer(&CS_BAR_PORT, CS_BAR_PIN, data_in, data_out, sizeof(data_in));
    
    // Set low noise mode
    data_in[0] = BAR_CTRL_REG2;
    data_in[1] = BAR_ADD_INC | BAR_LOW_NOISE;
    SPI_transfer(&CS_BAR_PORT, CS_BAR_PIN, data_in, data_out, sizeof(data_in));
    
    return BAR_Ready;
}
/*
BAR_Machine Calibrate_Barometer(BAR_Data *data){

}
*/
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
    
    uint32_t pressure_LSB = barometer.pressure_LSB_bytes[1] +
            (((uint16_t)barometer.pressure_LSB_bytes[2])<<8) + 
            (((uint32_t)barometer.pressure_LSB_bytes[3])<<16);
    barometer.pressure_pa = ((double)pressure_LSB)*sensitivity;
    barometer.height = c1*(pow(barometer.pressure_pa/standard_pressure, c2) - 1.0);
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