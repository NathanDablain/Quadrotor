#ifndef DMA_H
#define	DMA_H

#include <stdint.h>

#define DMA_SPI1RX_CHANNEL 0x06
#define DMA_SPI2RX_CHANNEL 0x08
#define DMA_DMA0_CHANNEL 0x5B
#define DMA_I2C1TX_CHANNEL 0x6D

void Initialize_DMA();

void Set_DMA_01(uint8_t* source_address, uint8_t* destination_address, uint32_t length);

void Set_DMA_2(uint8_t* source_address, uint32_t length);

void Set_DMA_3(uint8_t* source_address, uint32_t length);

#endif	

