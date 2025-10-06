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

bool Setup(){
    bool setup_status = true;
    INTCON1bits.GIE = 0;

    Setup_Pins();
    
    Initialize_Clocks();
    
    Initialize_Timer1();
        
    Initialize_ADC();

    Initialize_SPI();
    
    Initialize_DMA();    
    
    Run_Barometer_Machine();
    
    Run_IMU_Machine();
    
    Run_Magnetometer_Machine();
    
    Setup_LoRa();
    //---Enable peripherals---//
    
    //---Configure external devices--//

    //---Enable Interrupts---//
    
    INTCON1bits.GIE = 1;

    return setup_status;
}

int main(void) {

    bool setup_status = Setup();     
    
    while(setup_status){
        Execute();
    }
    
    return 0;
}

void Execute(){
    static bool A_Filter_Init = false;
    static Uplink uplink;
    
    Sample_Voltages();
    
    Run_Barometer_Machine();
    
    Run_IMU_Machine();
    
    Run_Magnetometer_Machine();

    Run_LORA(&uplink);
    
    Send_Pages();

    switch (g_Flight_Controller_Status){
        case Standby:
            Run_Ground_Filter(true);
            break;
            
        case User_Calibration:
            break;
            
        case System_Calibration:
            // Run when drone is in takeoff position to estimate gyro biases and initial rotation
            Run_Ground_Filter(false);
            
            break;
            
        case Ready:
            // Save off ground filter states as initial conditions for air filter
            Run_Air_Filter(true);
            break;
            
        case Flying:
            Run_Air_Filter(false);
            break;
            
        case Landing:
            Run_Air_Filter(false);
            break;
                    
    }

}