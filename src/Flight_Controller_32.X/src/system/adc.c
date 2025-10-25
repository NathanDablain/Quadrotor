#include "adc.h"
#include <xc.h>
#include <stdint.h>
#include "global_variables.h"

void Initialize_ADC(){
    // Channel 0 will be for monitoring the 1S LIPO voltage that powers the flight controller
    // Enable oversampling
    AD1CH0CONbits.MODE = 3;
    // Set to 64 samples, 15 bit result
    AD1CH0CONbits.ACCNUM = 2;
    // Enable continuous sample
    AD1CH0CONbits.ACCBRST = 1;
    // Select positive input
    AD1CH0CONbits.PINSEL = ADC_1S_MUX;
    // Select ground as negative input
    AD1CH0CONbits.NINSEL = 0;
    // Set sampling time to 181 ns
    AD1CH0CONbits.SAMC = 7;
    // Enable software trigger of sampling
    AD1CH0CONbits.TRG1SRC = 1;
    // Enable back to back samples
    AD1CH0CONbits.TRG2SRC = 2;
    
    // Channel 1 will be for monitoring the 4S LIPO voltage the powers the ESC and motors
    // Enable oversampling
    AD1CH1CONbits.MODE = 3;
    // Set to 64 samples, 15 bit result
    AD1CH1CONbits.ACCNUM = 2;
    // Enable continuous sample
    AD1CH1CONbits.ACCBRST = 1;
    // Select positive input
    AD1CH1CONbits.PINSEL = ADC_4S_MUX;
    // Select ground as negative input
    AD1CH1CONbits.NINSEL = 0;
    // Set sampling time to 181 ns
    AD1CH1CONbits.SAMC = 7;
    // Enable software trigger of sampling
    AD1CH1CONbits.TRG1SRC = 1;
    // Enable back to back samples
    AD1CH1CONbits.TRG2SRC = 2;
    
    // Channel 2 is for finding the reference voltage applied to AVCC
    AD1CH2CONbits.MODE = 3;
        // Set to 64 samples, 15 bit result
    AD1CH2CONbits.ACCNUM = 2;
    // Enable continuous sample
    AD1CH2CONbits.ACCBRST = 1;
    // Select positive input
    AD1CH2CONbits.PINSEL = ADC_15_16_AVCC_MUX;
    // Select ground as negative input
    AD1CH2CONbits.NINSEL = 0;
    // Set sampling time to 181 ns
    AD1CH2CONbits.SAMC = 7;
    // Enable software trigger of sampling
    AD1CH2CONbits.TRG1SRC = 1;
    // Enable back to back samples
    AD1CH2CONbits.TRG2SRC = 2;
    
    // Set interrupts
    IEC4bits.AD1CH0IE = 1;
    IEC4bits.AD1CH1IE = 1;
    
    // Enable ADC
    AD1CONbits.ON = 1;
    while(!AD1CONbits.ADRDY);
    
    // Initiate samples
    AD1SWTRGbits.CH0TRG = 1;
    AD1SWTRGbits.CH1TRG = 1;
    AD1SWTRGbits.CH2TRG = 1;
}

void Trigger_ADC(uint8_t channel){
    switch(channel){
        case ADC_1S_CHANNEL:
            AD1SWTRGbits.CH0TRG = 1;
            break;
        case ADC_4S_CHANNEL:
            AD1SWTRGbits.CH1TRG = 1;
            break;
        case 255:
            AD1SWTRGbits.CH0TRG = 1;
            AD1SWTRGbits.CH1TRG = 1;
            break;
    }

}

void _ISR _AD1CH0Interrupt(){
    IFS4bits.AD1CH0IF = 0;
    g_adc_1s_result = AD1CH0DATA;
}

void _ISR _AD1CH1Interrupt(){
    IFS4bits.AD1CH1IF = 0;
    g_adc_4s_result = AD1CH1DATA;
}
