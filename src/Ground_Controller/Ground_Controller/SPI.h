#ifndef SPI_H
#define SPI_H

// How long to delay when sending messages
#define SPI_TIMEOUT_THRESHOLD 100000UL
// Chip Select (CS) pins for devices connected via SPI
#define CS_LORA 0 // PC0
#define CS_BAR 7 // PA7
// IO Ports corresponding with the CS pins
#define PORT_LORA 'C'
#define PORT_BAR 'A'
// Pins for the Master Out Slave In (MOSI) and Serial Clock (SCK) portions of the interface
#define MOSI 4 // PA4
#define SCK 6 // PA6

// Initializes MOSI, SCK, and CS pins as outputs, enables master mode
void Setup_SPI();
// Reads one, or a stream of bytes from a given address
unsigned char Read_SPI(char Port, unsigned char Pin, unsigned char Register, unsigned char *Data, unsigned char Data_Length);
// Writes a single byte to a single register
unsigned char Write_SPI(char Port, unsigned char Pin, unsigned char Register, unsigned char Data);
// Writes a stream of character bytes to a given address
unsigned char Write_SPI_Stream(char Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length);

void Read_SPI_c(char Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length);

#endif