#include <xc.h>
#include "dma.h"
#include "pins.h"
#include "global_variables.h"
#include <stdint.h>
#include <stdbool.h>

void Initialize_DMA(){
    // DMA channel 0 will be used to load data into the SPI1 TX buffer,
    // it is triggered on a SPI1 receive buffer full interrupt
    // Increment source and keep destination unchanged
    DMA0CHbits.SAMODE = 1;
    DMA0CHbits.DAMODE = 0;
    // Set to one shot mode, size of one byte
    DMA0CHbits.TRMODE = 0;
    DMA0CHbits.SIZE = 0;
    // Set to trigger on SPI1 RX buffer interrupt
    DMA0SELbits.CHSEL = DMA_SPI1RX_CHANNEL;
    DMA0DST = (uint32_t) &SPI1BUF;

    // DMA channel 1 will be used to pull data from the SPI1 RX buffer,
    // it is triggered on a SPI1 receive buffer full interrupt
    // Increment destination and keep source unchanged
    DMA1CHbits.SAMODE = 0;
    DMA1CHbits.DAMODE = 1;
    // Set to one shot mode, size of one byte
    DMA1CHbits.TRMODE = 0;
    DMA1CHbits.SIZE = 0;
    // Enable interrupt upon completion
    DMA1CHbits.DONEEN = 1;
    IEC2bits.DMA1IE = 1;
    // Set to trigger on SPI1 RX buffer interrupt
    DMA1SELbits.CHSEL = DMA_SPI1RX_CHANNEL;
    DMA1SRC = (uint32_t) &SPI1BUF;
    
    // DMA channel 2 will be used to load data into the SPI2 TX buffer
    // it is triggered on a SPI2 receive buffer full interrupt
    // Increment source and keep destination unchanged
    DMA2CHbits.SAMODE = 1;
    DMA2CHbits.DAMODE = 0;
    // Set to one shot mode, size of one byte
    DMA2CHbits.TRMODE = 0;
    DMA2CHbits.SIZE = 0;
    // Enable interrupt upon completion
    DMA2CHbits.DONEEN = 1;
    IEC2bits.DMA2IE = 1;
    // Set to trigger on SPI2 RX buffer interrupt
    DMA2SELbits.CHSEL = DMA_SPI2RX_CHANNEL;
    DMA2DST = (uint32_t) &SPI2BUF;
    
    // DMA channel 3 will be used to load into the I2C1 TX buffer
    // it is triggered on a I2C1 transmit buffer empty interrupt
    // Increment source and keep destination unchanged
    DMA3CHbits.SAMODE = 1;
    DMA3CHbits.DAMODE = 0;
    // Set to one shot mode, size of one byte
    DMA3CHbits.TRMODE = 0;
    DMA3CHbits.SIZE = 0;
    // Enable interrupt upon completion
    DMA3CHbits.DONEEN = 1;
    IEC2bits.DMA3IE = 1;
    // Set to trigger on I2C1 TX buffer empty
    DMA3SELbits.CHSEL = DMA_I2C1TX_CHANNEL;
    DMA3DST = (uint32_t) &I2C1TRN;
    
    // Set limits of data space DMA can access, data space ranges from 0x4000 to 0x8000
    DMALOW = 0x4000;
    DMAHIGH = 0x8000;
    // Disable DMA module
    DMACONbits.ON = 1;
    
}

void Set_DMA_01(uint8_t* source_address, uint8_t* destination_address, uint32_t length){
    // Prevent access to SPI by other operations while transfer is in progress
    g_spi1_rdy_flag = false;
    
    // Destination already initialized as SPI1BUF, set source and count
    DMA0SRC = (uint32_t)source_address;
    DMA0CNT = length - 1;
    // Enable channel
    DMA0CHbits.CHEN = 1;
    // Source already initialized as SPI1BUF, set destination and count
    DMA1DST = (uint32_t)destination_address;
    DMA1CNT = length;
    // Enable channel
    DMA1CHbits.CHEN = 1;

    // Kick off SPI transfer
    LOWER_PIN(*g_spi1_reg_ptr, g_spi1_pin);
    SPI1BUF = *g_spi1_data_out_ptr;
}

void Set_DMA_2(uint8_t* source_address, uint32_t length){
    // Prevent access to SPI by other operations while transfer is in progress
    g_spi2_rdy_flag = false;
    
    // Destination already initialized as SPI2BUF, set source and count
    DMA2SRC = (uint32_t)source_address;
    DMA2CNT = length - 1;
    // Enable channel
    DMA2CHbits.CHEN = 1;

    // Kick off SPI transfer
    LOWER_PIN(*g_spi2_reg_ptr, g_spi2_pin);
    SPI2BUF = *g_spi2_data_out_ptr;
}

void Set_DMA_3(uint8_t* source_address, uint32_t length){
    // Prevent access to I2C1 by other operations while transfer is in progress
    g_i2c1_rdy_flag = false;
    
    // Destination already initialized as I2C1TRN, set source and count
    DMA3SRC = (uint32_t)source_address;
    DMA3CNT = length;
    // Enable channel
    DMA3CHbits.CHEN = 1;
    
    // Start I2C transfer
    I2C1CON1bits.SEN = 1;
    while (I2C1CON1bits.SEN);
    I2C1CON2bits.NDA = 0;
    I2C1TRN = ((uint32_t)g_i2c1_slave_address<<1U);
}

void _ISR _DMA1Interrupt(){
    // Raise SS pin, set global flags that transfer is complete, and disable channels
    RAISE_PIN(*g_spi1_reg_ptr, g_spi1_pin);
    g_spi1_rdy_flag = true;
    *g_spi1_transfer_done_flag = true;
    DMA0CHbits.CHEN = 0;
    DMA1CHbits.CHEN = 0;
    IFS2bits.DMA1IF = 0;
}

void _ISR _DMA2Interrupt(){
    // Raise SS pin, set global flags that transfer is complete, and disable channels
    RAISE_PIN(*g_spi2_reg_ptr, g_spi2_pin);
    g_spi2_rdy_flag = true;
    DMA2CHbits.CHEN = 0;
    IFS2bits.DMA2IF = 0;
}

void _ISR _DMA3Interrupt(){
    // Send stop command , set global flag that transfer is complete, disable channel
    I2C1CON1bits.PEN = 1; 
    g_i2c1_rdy_flag = true;
    DMA3CHbits.CHEN = 0;
    IFS2bits.DMA3IF = 0;
}