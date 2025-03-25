#ifndef GROUND_CONTROLLER_H
#define GROUND_CONTROLLER_H

// Standard libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/xmega.h>
#include <math.h>
#include <stdlib.h>

// Structures
typedef struct {
	unsigned char Led_pin;
	char Led_port;
} Led;

// Includes - specific
#include "SPI.h"
#include "TWI.h"
#include "Bar.h"
#include "SSD.h"
#include "LoRa.h"

// Function Declarations
unsigned char 
	Setup(void);
void
	Delay(unsigned long long length);

#endif