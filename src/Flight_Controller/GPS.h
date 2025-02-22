#ifndef GPS_H
#define GPS_H

#define GPS_WINDOW_SIZE 4

extern volatile unsigned char g_GPS_Read_Flag;

void Setup_GPS();
unsigned char Read_GPS(States *Drone);

#endif