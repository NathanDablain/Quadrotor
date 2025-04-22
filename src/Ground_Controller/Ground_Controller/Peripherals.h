#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <avr/io.h>
#include "GC_Types.h"

// Sets up timers used to control when functions are called
void Setup_Timers();
// Sets up Analog to Digital Converter, used to sample potentiometer output
void Setup_ADC();
// Sets up IO pins connected to buttons for interrupts, used to change statuses
void Setup_Buttons();
// Converts the digital reading of the potentiometer output to a floating point value used to control reference positions
float Set_dial_window(Dial *dial);
// Samples the ADC, returns the ADC reading
unsigned int Read_ADC(Dial_ID dial);

#endif