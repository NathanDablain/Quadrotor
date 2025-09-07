#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
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
//    static SEQUENCER sequencer = {0};
    static uint32_t count = 0;
    static Uplink uplink;
    static FC_Status Flight_Controller_Status = Standby;
    Print_Buffer print_buf = {0};
    print_buf.dble[0] = IMU_Angular_Rate(0);
    print_buf.dble[1] = IMU_Angular_Rate(1);
    print_buf.dble[2] = IMU_Angular_Rate(2);
    print_buf.dble[3] = IMU_Acceleration(0);
    print_buf.dble[4] = IMU_Acceleration(1);
    print_buf.dble[5] = IMU_Acceleration(2);

    Sample_Voltages();
    
    Run_Barometer_Machine();
    
    Run_IMU_Machine();
    
    Run_Magnetometer_Machine();

    Run_LORA(&uplink, &Flight_Controller_Status);

    Send_Pages(print_buf);
    if (g_seconds != count){
        count = g_seconds;
    }

}