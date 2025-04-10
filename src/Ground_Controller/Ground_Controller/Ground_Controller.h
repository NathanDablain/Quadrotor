#ifndef GROUND_CONTROLLER_H
#define GROUND_CONTROLLER_H

// Standard libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/xmega.h>
#include <math.h>
#include <stdlib.h>

// Includes - specific
#include "SPI.h"
#include "TWI.h"
#include "Bar.h"
#include "SSD.h"
#include "LoRa.h"
#include "GC_Types.h"

// Function Declarations
unsigned char Setup(void);

void Setup_Timers();

void Setup_ADC();

void Setup_Buttons();

void Set_dial_window(Dial *dial);

unsigned int Read_ADC(Dial_ID dial);

void Delay(unsigned long long length);

#endif