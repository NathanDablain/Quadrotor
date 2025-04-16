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

// Tracks when to sample barometer
extern volatile unsigned char g_BAR_Read_Flag;
// Tracks when to check for down links from drone
extern volatile unsigned char g_LoRa_Check_Flag;
// Tracks when to send up links to drone
extern volatile unsigned char g_LoRa_Uplink_Flag;

#define SETUP_SUCCESS 0x0F

// Contains all other setup function calls, sets CPU clock speed, enables interrupts, delays initialization on cold start
unsigned char Setup(void);
// Sets up timers used to control when functions are called
void Setup_Timers();
// Sets up Analog to Digital Converter, used to sample potentiometer output
void Setup_ADC();
// Sets up IO pins connected to buttons for interrupts, used to change statuses
void Setup_Buttons();
// Converts the digital reading of the potentiometer output to a floating point value used to control reference positions
void Set_dial_window(Dial *dial);
// Samples the ADC, returns the ADC reading
unsigned int Read_ADC(Dial_ID dial);
// Sets the Flight Controller (FC) status given current status
void Set_Status(Uplink *up_link);
// Prints to SSD displays
void Print_Displays(Dial *D_h, Dial *D_n, Dial *D_e, Uplink *up_link, Downlink_Reponse_Codes Downlink_Status);

#endif