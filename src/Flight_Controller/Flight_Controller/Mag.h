#ifndef MAG_H
#define	MAG_H

#include "FC_Types.h"

// Macros
#define MAG_WHO_AM_I 0x4F
#define MAG_CFG_REG_A 0x60
#define MAG_CFG_REG_B 0x61
#define MAG_CFG_REG_C 0x62
#define MAG_DATA_START 0x68
#define MAG_WINDOW_SIZE 16

// Functions
unsigned char 
	Setup_Mag(),
	Read_Mag(States *Drone);

#endif