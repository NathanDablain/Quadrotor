#ifndef SPI_H
#define SPI_H

// Includes
#include <avr/io.h>
#include <stdio.h>

// Macros
#define SPI_TIMEOUT_THRESHOLD 100000UL
#define CS_LORA 0 // PC0
#define CS_BAR 7 // PA7
#define PORT_LORA 'C'
#define PORT_BAR 'A'
#define MOSI 4 // PA4
#define SCK 6 // PA6

// Functions
void
	Setup_SPI();
unsigned char
	Read_SPI(char Port, unsigned char Pin, unsigned char Register, unsigned char *Data, unsigned char Data_Length),
	Write_SPI(char Port, unsigned char Pin, unsigned char Register, unsigned char Data);

#endif