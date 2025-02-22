#ifndef TWI_H
#define TWI_H

#define TWI_TIMEOUT_THRESHOLD 1e6

void Setup_TWI();
unsigned char Write_TWI(unsigned char Slave_Address, unsigned char Address_Byte, unsigned char *Data, unsigned char Data_Length);

#endif