#ifndef SPI_H
#define SPI_H

// Macros
#define SPI_TIMEOUT_THRESHOLD 100000
#define CS_LORA 5 // PA5
#define CS_IMU 6 // PA6
#define CS_BAR 7 // PA7
#define CS_DGW 2 // PB2
#define CS_MAG 3 // PB3
#define PORT_LORA 'A'
#define PORT_IMU 'A'
#define PORT_BAR 'A'
#define PORT_MAG 'B'
#define MOSI 0 // PC0
#define SCK 2 // PC2

// Functions
void Setup_SPI();

void Read_SPI_c(char Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length);

void Read_SPI(char Port, unsigned char Pin, unsigned char Register, unsigned char *Data, unsigned int Data_Length);

unsigned char Write_SPI(char Port, unsigned char Pin, unsigned char Register, unsigned char Data);
	
// Writes a stream of character bytes to a given address
unsigned char Write_SPI_Stream(char Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length);

#endif