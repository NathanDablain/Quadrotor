#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include "FC_Types.h"

#define MAX_THRUST 30.0

float Height_MRAC(States *MCU, float h_ref);

void Set_throttles(unsigned int motor_throttles[4], float desired_thrust, float desired_moments[3]);

void Attitude_Gain_Lookup(unsigned char index, float Gains[3][6]);

#endif