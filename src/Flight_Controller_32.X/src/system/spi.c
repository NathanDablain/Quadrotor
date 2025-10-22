#include <xc.h>
#include <stdbool.h>
#include "spi.h"
#include "pins.h"
#include "global_variables.h"

void Initialize_SPI(uint8_t index){
    if (index == 1){
        // Setup SPI1 for communication with mems devices and lora

        // Map SCK1 to RC7 (RP40)
        // Map SDO1 to RC9 (RP42)
        // Map SDI1 to RC6 (RP39)

        RPCONbits.IOLOCK = 0;
        _RP40R = 14;
        _RP42R = 13;
        _SDI1R = 39;
        RPCONbits.IOLOCK = 1;

        // Enable master mode, take clock from system clock
        SPI1CON1bits.MCLKEN = 0;
        SPI1CON1bits.MSTEN = 1;
        SPI1CON1bits.ENHBUF = 0;
        SPI1IMSKbits.SPIRBFEN = 1;
        // Enable RX interrupts when RX buffer is full
        IFS1bits.SPI1RXIF = 0;
        IEC1bits.SPI1RXIE = 1;
        // Set frequency based on -> F_SCK = F_SPI/(2*(SPI_BRG+1)
        // 10 MHz = 200MHz/(2*(9+1))
        SPI1BRG = 9;

        // Turn on module
        SPI1CON1bits.ON = 1;
    }
    else if (index == 2){
        // Setup SPI2 for communication with Arduino for oled display printing

        // Map SCK2 to RD3 (RP52)
        // Map SDO2 to RD1 (RP50)
        // Map SDI2 to RD2 (RP51)

        RPCONbits.IOLOCK = 0;
        _RP52R = 17;
        _RP50R = 16;
        _SDI2R = 51;
        RPCONbits.IOLOCK = 1;

        // Enable master mode, take clock from system clock
        SPI2CON1bits.MCLKEN = 0;
        SPI2CON1bits.MSTEN = 1;
        SPI2CON1bits.ENHBUF = 0;
        SPI2IMSKbits.SPIRBFEN = 1;
        // Enable RX interrupts when RX buffer is full
        IFS2bits.SPI2RXIF = 0;
        IEC2bits.SPI2RXIE = 1;
        // Set frequency based on -> F_SCK = F_SPI/(2*(SPI_BRG+1)
        // 1 MHz = 200MHz/(2*(99+1)), 
        SPI2BRG = 99;

        // Turn on module
        SPI2CON1bits.ON = 1;
    }
}

void Prepare_SPI1_For_DMA(volatile uint32_t *reg, uint8_t pin, uint8_t *first_byte, bool *completion_flag){
    g_spi1_reg_ptr = reg;
    g_spi1_pin = pin;
    g_spi1_data_out_ptr = first_byte;
    g_spi1_transfer_done_flag = completion_flag;
}

void Prepare_SPI2_For_DMA(volatile uint32_t *reg, uint8_t pin, uint8_t *first_byte){
    g_spi2_reg_ptr = reg;
    g_spi2_pin = pin;
    g_spi2_data_out_ptr = first_byte;
}

void SPI_transfer(volatile uint32_t *reg, uint8_t pin, uint8_t *data_in, uint8_t *data_out, uint8_t length){
    // This function allows easy transferring of data, but involves the cpu sitting in a while loop
    // So we will use this function for initialization of devices but not for ongoing data transfer during run time
    uint8_t counter = 0;
            
    LOWER_PIN(*reg, pin);
    
    while(counter < length){
        SPI1BUF = data_in[counter];
        while(SPI1STATbits.SPIBUSY);
        data_out[counter] = SPI1BUF;
        counter++;
    }

    RAISE_PIN(*reg, pin);    
}

void _ISR _SPI1RXInterrupt(){
    IFS1bits.SPI1RXIF = 0;
}

void _ISR _SPI2RXInterrupt(){
    volatile uint8_t trash = SPI2BUF;
    IFS2bits.SPI2RXIF = 0;
}