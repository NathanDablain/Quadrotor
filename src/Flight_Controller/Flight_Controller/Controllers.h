#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include "FC_Types.h"

#define MAX_THRUST 10.0 // N
#define MAX_MOMENT 2.0 // N-m
#define MAX_W 2.0 // rad/s
#define INDEX_NOT_SET 100

float Height_MRAC(States *Drone, float h_ref);

void Set_throttles(unsigned int motor_throttles[4], float desired_thrust, float desired_moments[3]);

void Run_Motors(unsigned int Throttle_Commands[4]);

void Attitude_Gain_Lookup(unsigned char index, float Gains[3][6]);

void Attitude_LQR(States *Drone, States *Reference);

void Angular_Rate_Control(States *Drone, States *Reference, float desired_moments[3]);

float Saturate(float value_in, float max_value, float min_value);

float Angle_difference(float angle1, float angle2);

float Angle_Discontinuity(float angle);

#endif