#ifndef BAR_H
#define BAR_H

// Protocol for 7th bit when using SPI interface:
// -> 0 is a write, 1 is a read

// Barometer address, only needed to check that device is working
#define BAR_WHO_AM_I 0x0F
// Barometer registers to be set when initializing device
#define BAR_IF_CTRL 0x0E
#define BAR_CTRL_REG1 0x10
#define BAR_CTRL_REG2 0x11
// Barometer register where the first byte of the two byte pressure data is
#define BAR_DATA_START 0x28
// Barometer sensitivity, converts LSB to Pascals
#define BAR_SENS 1.0/40.96
// Size of FIR, keep in powers of 2 to allow for quick division
#define BAR_WINDOW_SIZE 16
// Constants used to convert Pascals to height 
#define BAR_Tb 288.15 // Standard temperature at sea level (K)
#define BAR_g 9.8065 // Acceleration due to gravity (m/s^2)
#define BAR_Lb -0.0065 // Standard temperature lapse rate (K/m)
#define BAR_Pb 101325.0 // Standard static pressure at sea level (Pa)
#define BAR_R 8.31432 // Universal gas constant (J/(mol-K))S
#define BAR_M 0.0289644 // Molar mass of air (kg/mol)

// Checks ID, sets output data rate (ODR) and noise mode
unsigned char Setup_Bar();
// Reads in pressure data in LSB from barometer registers, if at FIR window limit, calls Height_Bar
unsigned char Read_Bar(float *pressure_altitude);
// Converts pressure in LSB to height in meters 
float Height_Bar(unsigned long pressure_LSB);

#endif