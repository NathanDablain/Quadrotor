#include <xc.h>
#include <stdint.h>
#include <limits.h>
#include "global_variables.h"
#include "adc.h"
#include "pins.h"

void Sample_Voltages(){
    static uint32_t Update_time_last = 0;
    static uint16_t AVCC_voltage = 0;
    static uint32_t ADC_max = SHRT_MAX;
    const float ADC_sensitivity = 3.3;
    uint32_t Voltage_1s_lsb;
    float Voltage_1S;
    
    // Once per second, sample the LIPO voltages
    if (g_seconds != Update_time_last){
        Update_time_last = g_seconds;
        
        Trigger_ADC();
        // Get the reference voltage
        if ((AVCC_voltage == 0) && (AD1STATbits.CH2RDY == 1)){
            AVCC_voltage = AD1CH2DATA;
            ADC_max = (AVCC_voltage*16);
            ADC_max /= 15;
        }
        
        // 1S LIPO voltage is read through a 10kohm-10kohm voltage divider, so real voltage is double whats measured
        Voltage_1s_lsb = g_adc_1s_result*2;
        Voltage_1S = (float)Voltage_1s_lsb/(float)ADC_max;
        Voltage_1S *= ADC_sensitivity;
        
        if (Voltage_1S >= 3.6){
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

