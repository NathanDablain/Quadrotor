#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "system_types.h"
#include "time.h"
#include "global_variables.h"
#include "system_clocks.h"
#include "adc.h"
#include "pins.h"
#include "spi.h"
#include "voltage_monitor.h"
#include "barometer.h"
#include "imu.h"
#include "magnetometer.h"
#include "dma.h"
#include "lora.h"
#include "i2c.h"
#include "oled.h"
#include "kalman_filter.h"
#include "navigation.h"
#include "guidance.h"
#include "controllers.h"

static const bool arduino_interchange = false;

bool Setup(){
    bool setup_status = true;
    INTCON1bits.GIE = 0;
    
    Initialize_Clocks();
    
    Setup_Pins();

    Initialize_Timer1();
        
    Initialize_ADC();

    Initialize_SPI(1);
    
    if (arduino_interchange){
        Initialize_SPI(2);
    }
    
    Initialize_DMA();    
    
    Initialize_Barometer_Machine();
    
    Initialize_IMU_Machine();
    
    Initialize_Magnetometer_Machine();
    
    Initialize_LORA_Machine();
    
    Manage_FC_Status(Standby);
    
    Initialize_Guidance_Machine();
    
    Initialize_Controllers();
    
    INTCON1bits.GIE = 1;

    return setup_status;
}

int main(void) {

    bool setup_status = Setup();     
    Manage_FC_Status(System_Calibration);

    while(setup_status){
        Execute();
    }
    
    return 0;
}

void Execute(){
   
    Sample_Voltages();
    
    Run_Barometer_Machine();
    
    Run_IMU_Machine();
    
    Run_Magnetometer_Machine();

    Run_LORA();
    
    if (arduino_interchange){
        Send_Pages();
    }
        
    if (g_Flight_Controller_Status == System_Calibration){
        Run_Ground_Filter(false);
    }
    else if (g_Flight_Controller_Status == Flying || g_Flight_Controller_Status == Landing){
        Guidance_Machine();
        Run_Air_Filter(false);
        Run_Altitude_Filter(false);
        Run_Controllers();
    }
    
}