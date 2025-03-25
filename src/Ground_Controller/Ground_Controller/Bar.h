#ifndef BAR_H
#define BAR_H

// Includes
#include <math.h>

// Macros
#define BAR_WHO_AM_I 0x0F
#define BAR_IF_CTRL 0x0E
#define BAR_CTRL_REG1 0x10
#define BAR_CTRL_REG2 0x11
#define BAR_DATA_START 0x28
#define BAR_SENS 1.0/40.96 // Pa/LSB
#define BAR_WINDOW_SIZE 16
#define BAR_Tb 288.15 // Standard temperature at sea level (K)
#define BAR_g 9.8065 // Acceleration due to gravity (m/s^2)
#define BAR_Lb -0.0065 // Standard temperature lapse rate (K/m)
#define BAR_Pb 101325.0 // Standard static pressure at sea level (Pa)
#define BAR_R 8.31432 // Universal gas constant (J/(mol-K))S
#define BAR_M 0.0289644 // Molar mass of air (kg/mol)

// Functions
unsigned char 
	Setup_Bar(),
	Read_Bar(float *pressure_altitude);
float Height_Bar(unsigned long pressure_LSB);

#endif