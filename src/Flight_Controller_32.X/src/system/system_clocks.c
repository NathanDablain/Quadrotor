#include <xc.h>
#include "system_clocks.h"
#include "global_variables.h"

void Initialize_Clocks(){
    // Enable the high speed primary oscillator
    // This gain is recommended for primary oscillator clocks of 8-16 MHz
    OSCCFGbits.GAIN = 1;
    OSCCFGbits.POSCMD = 2;
    OSCCTRLbits.POSCEN = 1;
    while (!OSCCTRLbits.POSCRDY);
    // POSC requires delay to start up properly
    Delay(100000);
    //---Set oscillator to 200 MHz---//
    // Recommended to change system clock from PLL if already running to avoid over clocking in setup
    if (CLK1CONbits.COSC != 1){
        CLK1CONbits.NOSC = 1;
        //Enable clock switching
        CLK1CONbits.OSWEN = 1;
        //Wait for clock switching complete
        while (CLK1CONbits.OSWEN);
    }
    
    // Turn on PLL 1 and enable output
    PLL1CONbits.ON = 1;
    PLL1CONbits.OE = 1;
    // Sets up BFRC as backup oscillator, enable fail safe monitoring
    PLL1CONbits.BOSC = 2;
    PLL1CONbits.FSCMEN = 1; 
    // From data sheet -> PLL Fout = Fin*FBDIV / (PLLPRE * POSTDIV1 * POSTDIV2)
    // We have Fin fixed at 12MHz from an external crystal
    // 200MHz = 12MHz*50 / (3 * 1 * 1)
    PLL1DIVbits.PLLFBDIV = 50;
    PLL1DIVbits.PLLPRE = 1;
    PLL1DIVbits.POSTDIV1 = 3;
    PLL1DIVbits.POSTDIV2 = 1;
    // Enable PLL update
    PLL1CONbits.PLLSWEN = 1;
    while (PLL1CONbits.PLLSWEN);
    // Enable fail safe monitoring
    PLL1CONbits.FOUTSWEN = 1;
    while (PLL1CONbits.FOUTSWEN);
    PLL1CONbits.FSCMEN = 1;
    // Select clock switching and set clock source to external POSC
    PLL1CONbits.NOSC = 3;
    PLL1CONbits.OSWEN = 1;
    while (PLL1CONbits.OSWEN);
    while (!OSCCTRLbits.PLL1RDY);
    
    // Clock 1 (System clock) settings
    CLK1CONbits.NOSC = 5; // Takes PLL1 FOUT as its clock source
    CLK1CONbits.BOSC = 2;
    CLK1CONbits.FSCMEN = 1;
    //Enable clock switching
    CLK1CONbits.OSWEN = 1;
    //Wait for clock switching complete
    while (CLK1CONbits.OSWEN);
    while (!CLK1CONbits.CLKRDY);
    
    // Clock 6 (ADC clock) settings
    CLK6CONbits.NOSC = 5; // Takes PLL1 FOUT as its clock source
    CLK6CONbits.BOSC = 2;
    CLK6CONbits.FSCMEN = 1;
    CLK6CONbits.ON = 1;
    //Enable clock switching
    CLK6CONbits.OSWEN = 1;
    //Wait for clock switching complete
    while (CLK6CONbits.OSWEN);
    while (!CLK6CONbits.CLKRDY);
    
    // Clock 9 (SPI clock) settings
    CLK9CONbits.NOSC = 1; // Takes FRC (8MHz) as its clock source
    CLK9CONbits.BOSC = 2;
    CLK9CONbits.FSCMEN = 1;
    CLK9CONbits.ON = 1;
    //Enable clock switching
    CLK9CONbits.OSWEN = 1;
    //Wait for clock switching complete
    while (CLK9CONbits.OSWEN);
    while (!CLK9CONbits.CLKRDY);
    
}

void Initialize_Timer1(){
    // We want timer1 to generate an interrupt once per second
    g_seconds = 0;
    // Set div8 prescalar
    T1CONbits.TCKPS = 1; 
    T1CONbits.TCS = 0;
    TMR1 = 0;
    PR1 = g_tmr1_ct_in_s;
    // Enable Timer1 interrupts
    IEC1bits.T1IE = 1;
    // Enable timer
    T1CONbits.ON = 1;
}

void _ISR _T1Interrupt(void){
    IFS1bits.T1IF = 0;
    ++g_seconds;
    TMR1 = 0;
}
