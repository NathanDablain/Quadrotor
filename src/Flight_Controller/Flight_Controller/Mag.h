#ifndef MAG_H
#define	MAG_H

#include "FC_Types.h"

#define MAG_WHO_AM_I 0x4F
#define MAG_CFG_REG_A 0x60
#define MAG_CFG_REG_B 0x61
#define MAG_CFG_REG_C 0x62
#define MAG_DATA_START 0x68
#define MAG_STATUS 0x67
#define MAG_DRDY_bm (1<<3)
#define MAG_WINDOW_SIZE 4
#define MAG_CAL_TIMEOUT 5

unsigned char Setup_Mag();

unsigned char Read_Mag(States *Drone, Calibration_Data *cal_data);

void Calibrate_Mag(States *Drone, Calibration_Data *cal_data);

#endif