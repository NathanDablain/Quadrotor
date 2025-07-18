#ifndef SPI_H
#define SPI_H

void Setup_SPI();

void Read_SPI_c(volatile register8_t *Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length);

void Read_SPI(volatile register8_t *Port, unsigned char Pin, unsigned char Register, unsigned char *Data, unsigned int Data_Length);

void Write_SPI(volatile register8_t *Port, unsigned char Pin, unsigned char Register, unsigned char Data);
	
void Write_SPI_Stream(volatile register8_t *Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length);

#endif