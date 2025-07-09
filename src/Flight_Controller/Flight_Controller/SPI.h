#ifndef SPI_H
#define SPI_H

// Macros
#define SPI_TIMEOUT_THRESHOLD 100000

#define MOSI 0 // PC0
#define SCK 2 // PC2

// Functions
void Setup_SPI();

void Read_SPI_c(volatile unsigned char *Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length);

void Read_SPI(volatile unsigned char *Port, unsigned char Pin, unsigned char Register, unsigned char *Data, unsigned int Data_Length);

unsigned char Write_SPI(volatile unsigned char *Port, unsigned char Pin, unsigned char Register, unsigned char Data);
	
// Writes a stream of character bytes to a given address
unsigned char Write_SPI_Stream(volatile unsigned char *Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length);

#endif