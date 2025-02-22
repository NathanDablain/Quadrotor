#ifndef MAIN_H
#define MAIN_H

// Includes - generic
#include <avr/io.h>
#include <avr/xmega.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Global variables
extern volatile unsigned long g_seconds;

// Macros
#define SET_BIT(current_val, position, val) ((val < 1) ? current_val&(~(val<<position)) : current_val|(val<<position))
#define D2R M_PI/180.0

// Function Declarations
unsigned char 
	Setup();
void 
	Run(unsigned char Setup_Bitmask),
	Delay(unsigned long long length);

// Structures
typedef struct{
	float w[3];
	float g_vec[3]; // In the frame Forward - Right - Down
	float m_vec[3];
	float Euler[3];
	float pressure_altitude;
	float Position_NED[3];
	signed long Longitude;
	signed long Latitude;
	float Position_ECEF[3];
	float Speed_over_ground;
	float Course_over_ground;
} States;

// Includes - specific
#include "SPI.h"
#include "TWI.h"
#include "IMU.h"
#include "Mag.h"
#include "Bar.h"
#include "Observer.h"
#include "GPS.h"
#include "SSD.h"
#include "LoRa.h"


#endif