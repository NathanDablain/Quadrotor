#ifndef GPS_H
#define GPS_H

#define GPS_WINDOW_SIZE 4

extern volatile unsigned char g_GPS_Read_Flag;

void 
	USART_Transmit(char* Message, unsigned char length);
unsigned char
	Setup_GPS(),
	Read_GPS(States *Drone),
	LLA_to_NED(signed long Latitude, signed long Longitude, float Height, float Position_NED[3]);

#endif