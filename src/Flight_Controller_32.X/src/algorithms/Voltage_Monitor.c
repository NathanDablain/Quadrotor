#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "voltage_monitor.h"
#include "global_variables.h"
#include "time.h"
#include "adc.h"
#include "pins.h"
#include "motors.h"

void Sample_Voltages(){
    const Time Fast_Sample_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/1000};
    
    static Time     Last_Sample_Time   = {0};
    static bool     Motors_Initialized = true;
    static uint32_t Update_time_last   = 0;
    static uint16_t AVCC_voltage       = 0;
    static uint32_t ADC_max            = SHRT_MAX;
    
    uint32_t Voltage_1s_lsb;
    uint32_t Voltage_4s_lsb;
    float    Voltage_1S;
    float    Voltage_4S;
    
    // When we are in system calibration, need to rapidly sample 4S to be able
    // to quickly trigger motor calibration when battery is connected
    if (g_Flight_Controller_Status == System_Calibration && !Motors_Initialized){
        if (Compare_And_Update(Current_Time(), Fast_Sample_Rate, &Last_Sample_Time)){
            Trigger_ADC(ADC_4S_CHANNEL);
            
            // 4S LIPO voltage is read through a 47kohm-10kohm voltage divider,
            // so real voltage is 5.7 times whats measured
            Voltage_4s_lsb = g_adc_4s_result * SENSITIVITY_4S;
            Voltage_4S = (float)Voltage_4s_lsb/(float)ADC_max;
            Voltage_4S *= SENSITIVITY_ADC;
            
            if (Voltage_4S >= MOTOR_CAL_THRESHOLD){
                Calibrate_Motors();
                Motors_Initialized = true;
            }
        }
    }

    // Once per second, sample the 1S LIPO voltages
    if (g_seconds != Update_time_last){
        Update_time_last = g_seconds;
        
        Trigger_ADC(ADC_1S_CHANNEL);
        // Get the reference voltage
        if ((AVCC_voltage == 0) && (AD1STATbits.CH2RDY == 1)){
            AVCC_voltage = AD1CH2DATA;
            ADC_max = (AVCC_voltage*16);
            ADC_max /= 15;
        }
        
        // 1S LIPO voltage is read through a 10kohm-10kohm voltage divider,
        // so real voltage is double whats measured
        Voltage_1s_lsb = g_adc_1s_result * SENSITIVITY_1S;
        Voltage_1S = (float)Voltage_1s_lsb/(float)ADC_max;
        Voltage_1S *= SENSITIVITY_ADC;
        
        if (Voltage_1S >= RED_LED_THRESHOLD){
            // Illuminate white LED
            RAISE_PIN(WHITE_LED_PORT, WHITE_LED_PIN);
            LOWER_PIN(RED_LED_PORT, RED_LED_PIN);
        }
        else {
            // Illuminate red LED
            LOWER_PIN(WHITE_LED_PORT, WHITE_LED_PIN);
            RAISE_PIN(RED_LED_PORT, RED_LED_PIN);
        }
    }
}

