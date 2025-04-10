#ifndef TWI_H
#define TWI_H

#include <avr/io.h>

#define TWI_TIMEOUT_THRESHOLD 1000000UL

void Setup_TWI();

unsigned char Write_TWI0(unsigned char Slave_Address, unsigned char Address_Byte, unsigned char *Data, unsigned char Data_Length);

unsigned char Write_TWI1(unsigned char Slave_Address, unsigned char Address_Byte, unsigned char *Data, unsigned char Data_Length);


#endif