#ifndef IMU_H
#define IMU_H

// Macros
#define IMU_WHO_AM_I 0x0F
#define IMU_CTRL1_XL 0x10
#define IMU_CTRL2_G 0x11
#define IMU_CTRL8_XL 0x17
#define IMU_DATA_START 0x22
#define IMU_WINDOW_SIZE 8 // Size of FIR window
#define GYRO_SENS 500.0/32768.0
#define ACCEL_SENS 2.0/32768.0

// Functions
unsigned char 
	Setup_IMU(),
	Read_IMU(States *Drone);

#endif