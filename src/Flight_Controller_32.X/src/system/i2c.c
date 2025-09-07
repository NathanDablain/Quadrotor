#include <xc.h>
#include <stdint.h>
#include <p32AK1216GC41064.h>
#include "i2c.h"
#include "pins.h"

void Initialize_I2C(){
    // RB3 is mapped to SDA1 by default
    // RB4 is mapped to SCL1 by default
    // Acknowledge all bytes including the end of packet
    _I2C1IE = 0U;
    _I2C1EIE = 0U;
    
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    // Enable smart mode
    // LBRG/HBGR of 207 at 200MHz system clock produces 400kHz I2C clock rate
    // Registers must have a minimum value of 4
    I2C1LBRGbits.I2CLBRG = 957UL;
    I2C1HBRGbits.I2CHBRG = 957UL;

    // Enable module
    I2C1CON1bits.SCLREL = 1;
    I2C1CON1bits.SDAHT = 0;
    I2C1CON1bits.ON = 1U;
    
    /* Clear host interrupt flag */
    _I2C1IF = 0U;

    /* Clear fault interrupt flag */
    _I2C1EIF = 0U;

}

uint8_t Write_I2C(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint8_t data_length){
    // This function will hold the cpu in while loops waiting for slave response, use this for external device initialization only
    uint8_t data_counter = 0;
    // Sends start command
//    _I2C1IE = 1U;
//    _I2C1EIE= 1U;
    I2C1CON1bits.SEN = 1;

    while (I2C1STAT1bits.TBF);
    if (I2C1STAT1bits.ACKSTAT) return 0;
    I2C1CON2bits.NDA = 0;
    I2C1TRN = ((uint32_t)slave_address<<1U);

    while (I2C1STAT1bits.TBF);
    if (I2C1STAT1bits.ACKSTAT) return 0;
    I2C1CON2bits.NDA = 1;
    I2C1TRN = register_address;

    while (I2C1STAT1bits.TBF);
    if (I2C1STAT1bits.ACKSTAT) return 0;
    while (data_counter++ < data_length){
        I2C1TRN = *data++;
        while (I2C1STAT1bits.TBF);
        if (I2C1STAT1bits.ACKSTAT) return 2;
    }
    // Send stop command
    I2C1CON1bits.PEN = 1;
    return 4;
}