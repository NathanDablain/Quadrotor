#include "main.h"
#include "time.h"
#include "linear_algebra.h"
#include "kalman_filter.h"
#include "global_variables.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <xc.h>
#include <stdbool.h>


int main(void) {
    static SEQUENCER sequencer = {0};
    bool time_to_update = Compare_And_Update(Current_Time(), g_gyro_sample_rate, &sequencer.imu_clk);
    bool setup_status = Setup();
    
    while(setup_status){
        Execute();
    }
    
    return 0;
}

bool Setup(){
    bool setup_status = true;
    //---Set oscillator to 200 MHz---//
    // Below setting are recommended for external clock of 12 MHz
    OSCCFGbits.GAIN = 1;
    OSCCFGbits.POSCMD = 2;
    // Turn on PLL and enable output
    PLL1CONbits.ON = 1;
    PLL1CONbits.OE = 1;
    // Sets up BFRC as backup oscillator, enable fail safe monitoring
    PLL1CONbits.BOSC = 2;
    PLL1CONbits.FSCMEN = 1; 
    // From data sheet -> PLL Fout = Fin*FBDIV / (PLLPRE * POSTDIV1 * POSTDIV2)
    // We have Fin fixed at 12MHz from an external crystal
    // 200MHz = 12MHz*100 / (6 * 1 * 1)
    PLL1DIVbits.PLLFBDIV = 100;
    PLL1DIVbits.PLLPRE = 6;
    PLL1DIVbits.POSTDIV1 = 1;
    PLL1DIVbits.POSTDIV2 = 1;
    // Enable PLL update
    PLL1CONbits.PLLSWEN = 1;
    while(PLL1CONbits.PLLSWEN);
    // Enable fail safe monitoring
    PLL1CONbits.FSCMEN = 1;
    // Select clock switching clock source
    PLL1CONbits.NOSC = 3;
    PLL1CONbits.OSWEN = 1;
    while(PLL1CONbits.OSWEN);
    while(!OSCCTRLbits.PLL1RDY);

    //---Setup pins---//
    
    //---Enable peripherals---//
    // We want timer1 to generate an interrupt once per second
    g_seconds = 0;
    // Set div8 prescalar
    T1CONbits.TCKPS = 1; 
    T1CONbits.TCS = 0;
    TMR1 = 0;
    PR1 = g_tmr1_ct_in_s;
    // Enable Timer1 interrupts
    IFS1bits.T1IF = 1;
    IEC1bits.T1IE = 1;
    // Enable timer
    T1CONbits.ON = 1;
    //---Configure external devices--//

    //---Enable Interrupts---//
    INTCON1bits.GIE = 1;
    return setup_status;
}

void Execute(){
    static SEQUENCER sequencer = {0};
    static uint32_t count = 0;
//    if (Compare_And_Update(Current_Time(), &sequencer.imu_clk)){
//        count++;
//    }
}