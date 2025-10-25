#include <xc.h>
#include <stdint.h>
#include <p32AK1216GC41064.h>
#include "pins.h"
#include "motors.h"

void Setup_Pins(){
    // LED Pins
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB8 = 0;
    LOWER_PIN(WHITE_LED_PORT, WHITE_LED_PIN);
    LOWER_PIN(RED_LED_PORT, RED_LED_PIN);

    // ADC Pins
    TRISAbits.TRISA2 = 1;
    TRISAbits.TRISA9 = 1;
    ANSELAbits.ANSELA2 = 1;
    ANSELAbits.ANSELA9 = 1;
    
    // MEMS Pins
    TRISCbits.TRISC8 = 0;
    TRISBbits.TRISB11 = 0;
    TRISAbits.TRISA6 = 0;
    RAISE_PIN(CS_BAR_PORT, CS_BAR_PIN);
    RAISE_PIN(CS_MAG_PORT, CS_MAG_PIN);
    RAISE_PIN(CS_IMU_PORT, CS_IMU_PIN);
    
    // LORA Pins
    TRISDbits.TRISD7 = 0;
    RAISE_PIN(CS_LORA_PORT, CS_LORA_PIN);
    TRISDbits.TRISD12 = 1;

    // Arduino Pin
    TRISBbits.TRISB1 = 0;
    RAISE_PIN(CS_ARDUINO_PORT, CS_ARDUINO_PIN);
    
    // PWM pins
    // MCLKSEL ->1 CKLGEN5, ->0 Standard Speed Peripheral Clock
    // PWM Source Selection ->011 PWM 4, 010 PWM 3, 001 PWM 2, 000 PWM 1
    // TRIGB sets the start of PWMxL duty cycle, TRIGA sets the end of duty cycle
    // DC sets the start of PWMxH duty cycle, PHASE sets the end of duty cycle
    // SOC and EOC determine the period
    // Duty cycle controls how much of the period the pin is high for
    // Oneshot125 has duty cycle between 125us to 250us and a maximum update rate of 4kHz
    // We will use a 3kHz update rate to coincide with our control loop
    // Use CLKGEN5 with the FRC to produce a 8MHz clock
    PG3CONbits.ON = 0;    
    PG4CONbits.ON = 0;

    PCLKCONbits.MCLKSEL = 1;
    
    // Enable output on pins, independent mode, and active low
    PG4IOCONbits.PMOD = 0b01;
    PG4IOCONbits.PENH = 1;
    PG4IOCONbits.PENL = 1;
    PG4IOCONbits.POLH = 1;
    PG4IOCONbits.POLL = 1;
    
    PG3IOCONbits.PMOD = 0b01;
    PG3IOCONbits.PENH = 1;
    PG3IOCONbits.PENL = 1;
    PG3IOCONbits.POLH = 1;
    PG3IOCONbits.POLL = 1;
  
    // Set period to 334 us (3kHz) 
    MPER   = ((334*8)<<4);
    MDC    = 0;
    MPHASE = 0;
    // Enable dual output, independent edge mode.
    // Take period from MPER, take clock from MCLKSEL. Enable module
    PG4CONbits.MODSEL  = 0b010;
    PG4CONbits.CLKSEL  = 1;
    PG4CONbits.MPERSEL = 1;
    
    PG3CONbits.MODSEL = 0b010;
    PG3CONbits.CLKSEL = 1;
    PG3CONbits.MPERSEL = 1;
    
    // Keep start of duty cycle at 0
    PG4TRIGA = (MIN_DUTY_CYCLE<<4);
    PG4TRIGB = (0<<4);
    PG4PHASE = (MIN_DUTY_CYCLE<<4);
    PG4DC    = (0<<4);
    
    PG3TRIGA = (MIN_DUTY_CYCLE<<4);
    PG3TRIGB = (0<<4);
    PG3PHASE = (MIN_DUTY_CYCLE<<4);
    PG3DC    = (0<<4);
    
    PG4CONbits.ON      = 1;
    PG4STATbits.UPDREQ = 1;
       
    PG3CONbits.ON      = 1;
    PG3STATbits.UPDREQ = 1;

}
