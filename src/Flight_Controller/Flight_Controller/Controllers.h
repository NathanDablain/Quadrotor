#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "LoRa.h"
#include "FC_Types.h"

#define INDEX_NOT_SET 100
#define MOTOR_CUTOFF_ANGLE (45.0*D2R)
#define MOTOR_CUTOFF_ALTITUDE 10.0f

typedef struct {
	unsigned char Pins[4];
	unsigned char index;
} Motors;

float Altitude_Control(float h, float h_ref);

void Euler_Control(float Current_Euler[3], float Commanded_Euler[3], float desired_moments[3]);

void Set_throttles(unsigned int motor_throttles[4], float desired_thrust, float desired_moments[3]);

void Run_Motors(unsigned int Throttle_Commands[4]);

void Run_Guidance(Reference *Desired_States, Reference *Commanded_States);

void Safety_Check(States *Drone, unsigned int motor_throttles[4], FC_Status *Flight_Controller_Status);

void Calibrate_Motors(Calibration_Data *cal_data, unsigned int Motor_Throttles[4]);

#endif