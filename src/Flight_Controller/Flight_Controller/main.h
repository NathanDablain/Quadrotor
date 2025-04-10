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

// Macros
#define SET_BIT(current_val, position, val) ((val < 1) ? current_val&(~(val<<position)) : current_val|(val<<position))
#define NAV_GPS_bp 0
#define NAV_BAR_bp 1
#define NAV_IMU_bp 2
#define NAV_MAG_bp 3
#define NAV_LORA_bp 4
#define SU_SSD_bp 5
#define NAV_SENSORS_bm 0x1E //0x1F

// Function Declarations
unsigned char 
	Setup();
void
	Setup_Timers(),
	Run(unsigned char Setup_Bitmask);

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
#include "Controllers.h"
#include "FC_Types.h"

#endif