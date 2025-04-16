#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <avr/io.h>
#include <avr/xmega.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "SPI.h"
#include "TWI.h"
#include "IMU.h"
#include "Mag.h"
#include "Bar.h"
#include "Observer.h"
#include "GPS.h"
#include "SSD.h"
#include "LoRa.h"
#include "Utilities.h"
#include "Motors.h"
#include "Controllers.h"
#include "FC_Types.h"

// Tracks when to check LoRa for uplink, incremented at 200 Hz
extern volatile unsigned char g_LoRa_Check_Flag;
// Tracks when to print ouput to SSD display
extern volatile unsigned char g_Print_Flag;


// Either sets or clears a bit within a bitmask depending on the input
#define SET_BIT(current_val, position, val) ((val < 1) ? current_val&(~(val<<position)) : current_val|(val<<position))
// Bit positions within navigation bitmask to signal if a given sensor is working
#define NAV_GPS_bp 0
#define NAV_BAR_bp 1
#define NAV_IMU_bp 2
#define NAV_MAG_bp 3
#define NAV_LORA_bp 4
#define SU_SSD_bp 5
// Bit mask result when all sensors are operating correctly
#define NAV_SENSORS_bm 0x1F
#define CALIBRATION_COMPLETE 0x1E

unsigned char Setup();
// Initializes timers used to keep track events
void Setup_Timers();

#endif