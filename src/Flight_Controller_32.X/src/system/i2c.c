#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <p32AK1216GC41064.h>
#include "i2c.h"
#include "global_variables.h"
#include "pins.h"

void Initialize_I2C(){
    // SCL is on RD5 , RP54, ASCL1
    // SDA is on RD6 , RP55, ASDA1
    I2C1CON1bits.ON = 0;
    
    // Disable interrupts
    IEC2bits.I2C1TXIE = 0;
    IEC2bits.I2C1IE = 0;
    IEC2bits.I2C1EIE = 0;
    
    // LBRG/HBGR of 57 at 200MHz system clock produces 1MHz I2C clock rate
    // Slew rate control must be disabled in this mode, (1 is disabled)
    I2C1CON1bits.DISSLW = 1;
    I2C1LBRGbits.I2CLBRG = 57UL;
    I2C1HBRGbits.I2CHBRG = 57UL;

    // Enable timeouts on bus idle
    I2C1CON2bits.BITE = 1;
    I2C1BITObits.BITOTMR = 5000;
    I2C1STAT2bits.BITO = 0;
    
    
    // Set TXIF when TBF is cleared
    I2C1INTCbits.TXIE = 1;
    // Set IF when bus timeout occurs
    I2C1INTCbits.BITIE = 1;
    // Set EIF when error occurs
    I2C1INTCbits.BSCLTIE = 1;
    I2C1INTCbits.CBCTIE = 1;
    I2C1INTCbits.HBCTIE = 1;
    I2C1INTCbits.FRMEIE = 1;
    I2C1INTCbits.CRCIE = 1;
    I2C1INTCbits.BCLIE = 1;
    I2C1INTCbits.NACKIE = 1;
    
    // Enable module
    I2C1CON1bits.SCLREL = 1;
    I2C1CON1bits.SDAHT = 0;
    I2C1CON1bits.ON = 1;
    
    IFS2bits.I2C1EIF = 0;
    IFS2bits.I2C1IF = 0;
    IFS2bits.I2C1TXIF = 0;
    
    // Enable transmit interrupts
    IEC2bits.I2C1TXIE = 1;
    // Enable generic interrupts
    IEC2bits.I2C1IE = 1;
    // Enables error interrupts
    IEC2bits.I2C1EIE = 1;

}

uint8_t Write_I2C(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint8_t data_length){
    // This function will hold the cpu in while loops waiting for slave response, use this for external device initialization only
    uint8_t data_counter = 0;
    while(I2C1CON1bits.SEN || I2C1CON1bits.RSEN || I2C1CON1bits.PEN || I2C1CON1bits.RCEN || I2C1CON1bits.ACKEN);
    // Sends start command
    I2C1CON1bits.SEN = 1;

    while (I2C1CON1bits.SEN);
    I2C1CON2bits.NDA = 0;
    I2C1TRN = ((uint32_t)slave_address<<1U);

    while (I2C1STAT1bits.TBF){
        // Check for bus idle or NACK from slave to exit early
        if (I2C1STAT1bits.ACKSTAT || g_oled_fail_flag){
            g_oled_fail_flag = false;
            return 0;
        }
    };

    I2C1CON2bits.NDA = 1;
    I2C1TRN = register_address;

    while (I2C1STAT1bits.TBF){
        // Check for bus idle or NACK from slave to exit early
        if (I2C1STAT1bits.ACKSTAT || g_oled_fail_flag){
            g_oled_fail_flag = false;
            return 0;
        }
    };
    
    while (data_counter < data_length){
        I2C1TRN = *data++;
        while (I2C1STAT1bits.TBF){
            // Check for bus idle or NACK from slave to exit early
            if (I2C1STAT1bits.ACKSTAT || g_oled_fail_flag){
                g_oled_fail_flag = false;
                return 0;
            }
        }
        data_counter++;
    }
    // Send stop command
    I2C1CON1bits.PEN = 1;
    I2C1STAT2bits.EOP = 1;
    return 1;
}

void _ISR _I2C1TXInterrupt(){
    if (I2C1CON2bits.NDA == 0){
        I2C1CON2bits.NDA = 1;
    }
    IFS2bits.I2C1TXIF = 0;
}

void _ISR _I2C1Interrupt(){
    if (I2C1STAT2bits.BITO){
        g_oled_fail_flag = true;
    }
    IFS2bits.I2C1IF = 0;
}

void _ISR _I2C1EInterrupt(){
    g_oled_fail_flag = true;
    IFS2bits.I2C1EIF = 0;
}