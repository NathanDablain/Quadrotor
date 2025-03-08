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
#define NAV_GPS_bp 0
#define NAV_BAR_bp 1
#define NAV_IMU_bp 2
#define NAV_MAG_bp 3
#define NAV_LORA_bp 4
#define SU_SSD_bp 5
#define NAV_SENSORS_bm 0x1F

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
#include "Motors.h"

inline void Setup_Timers(){
	//-Setup Real Time Clock for keeping track of total run time-//
	RTC_CTRLA |= RTC_CORREN_bm | RTC_RTCEN_bm;
	RTC_INTCTRL |= RTC_CMP_bm;
	RTC_CMP = 32768;
	//----------------------------------------------------------//
	//--------Setup Timer/Counter A0 for output compare---------//
	// Is triggered every 10 ms, is used by:
	//  -> Motors
	TCA0_SINGLE_CTRLA |= TCA_SINGLE_CLKSEL_DIV8_gc;
	TCA0_SINGLE_INTCTRL |= TCA_SINGLE_CMP0_bm;
	//---------------------------------------------------------//
	//-------Setup Timer/Counter B0 for output compare---------//
	// Generates an interrupt every 5 ms, is used by:
	//  -> Motors running at 100 Hz
	//	-> Magnetometer running at 100 Hz
	//  -> Barometer running at 75 Hz
	//	-> Attitude observer running at 25 Hz
	//	-> Print statements, variable frequency
	TCB0_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV2_gc; // Enables timer, uses main clock with a prescaler of two
	TCB0_INTCTRL |= TCB_CAPT_bm; // Enables interrupt on capture
	TCB0_CCMP = 60000; // Value at which timer generates interrupt and resets
	//--------------------------------------------------------//
	//-------Setup Timer/Counter B1 for output compare--------//
	// Generates an interrupt every 4.807 ms, is used by:
	//	-> IMU running at 208 Hz
	TCB1_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV2_gc;
	TCB1_INTCTRL |= TCB_CAPT_bm;
	TCB1_CCMP = 57693;
	//-------------------------------------------------------//
}

#endif