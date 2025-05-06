#ifndef GPS_H
#define GPS_H

#include "FC_Types.h"

#define GPS_WINDOW_SIZE 4

extern volatile unsigned char g_GPS_Read_Flag;

void USART_Transmit(char* Message, unsigned char length);

unsigned char Setup_GPS();

void Read_GPS(States *Drone, Calibration_Data *cal_data);

void Calibrate_GPS(States *Drone, Calibration_Data *cal_data);

void LLA_to_NED(signed long Latitude, signed long Longitude, float Position_NED[3], float Reference_Position_ecef[3]);

#endif