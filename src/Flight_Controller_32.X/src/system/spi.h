#ifndef SPI_H
#define	SPI_H

#include <stdbool.h>
#include <stdint.h>

void Initialize_SPI();

void SPI_transfer(volatile uint32_t *reg, uint8_t pin, uint8_t *data_in, uint8_t *data_out, uint8_t length);

void Prepare_SPI1_For_DMA(volatile uint32_t *reg, uint8_t pin, uint8_t *first_byte, bool *completion_flag);

void Prepare_SPI2_For_DMA(volatile uint32_t *reg, uint8_t pin, uint8_t *first_byte);

void SPI_set(volatile uint32_t *reg, uint8_t pin, bool *completion_flag, uint8_t *data_out, uint8_t *data_in, uint8_t data_length);

#endif

