#ifndef GROUND_CONTROLLER_H
#define GROUND_CONTROLLER_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/xmega.h>
#include <stdlib.h>
#include <string.h>

#include "Bar.h"
#include "SSD.h"
#include "SPI.h"
#include "LoRa.h"
#include "Utilities.h"
#include "GC_Types.h"
#include "Peripherals.h"

// Tracks when to sample barometer
extern volatile unsigned char g_BAR_Read_Flag;
// Tracks when to check for down links from drone
extern volatile unsigned char g_LoRa_Check_Flag;
// Tracks when to send up links to drone
extern volatile unsigned char g_LoRa_Uplink_Flag;
// Tracks when to read the ADC
extern volatile unsigned char g_ADC_Flag;
// Tracks when button 0 has been pressed
extern volatile unsigned char g_Button0_Flag;
// Tracks how often to change button status
extern volatile unsigned char g_Button_Read_Flag;
// Tracks when to print
extern volatile unsigned char g_Print_Flag;

#define SETUP_SUCCESS 0x0F

// Contains all other setup function calls, sets CPU clock speed, enables interrupts, delays initialization on cold start
unsigned char Setup(void);

#endif