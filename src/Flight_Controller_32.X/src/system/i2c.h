#ifndef I2C_H
#define	I2C_H

#include <stdint.h>

void Initialize_I2C();

uint8_t Write_I2C(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint8_t data_length);

#endif	

