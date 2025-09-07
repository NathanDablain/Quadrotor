#include <xc.h>
#include <stdint.h>
#include <p32AK1216GC41064.h>
#include "pins.h"

void Setup_Pins(){
    // LED Pins
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;
    LOWER_PIN(GREEN_LED_PORT, GREEN_LED_PIN);
    LOWER_PIN(YELLOW_LED_PORT, YELLOW_LED_PIN);
    LOWER_PIN(RED_LED_PORT, RED_LED_PIN);

    // ADC Pins
    TRISAbits.TRISA2 = 1;
    TRISAbits.TRISA9 = 1;
    ANSELAbits.ANSELA2 = 1;
    ANSELAbits.ANSELA9 = 1;
    
    // MEMS Pins
    TRISCbits.TRISC8 = 0;
    TRISBbits.TRISB10 = 0;
    TRISBbits.TRISB11 = 0;
    RAISE_PIN(CS_BAR_PORT, CS_BAR_PIN);
    RAISE_PIN(CS_MAG_PORT, CS_MAG_PIN);
    RAISE_PIN(CS_IMU_PORT, CS_IMU_PIN);
    
    // LORA Pins
    TRISDbits.TRISD7 = 0;
    RAISE_PIN(CS_LORA_PORT, CS_LORA_PIN);
    TRISDbits.TRISD6 = 1;

    // Arduino Pin
    TRISBbits.TRISB1 = 0;
    RAISE_PIN(CS_ARDUINO_PORT, CS_ARDUINO_PIN);
}