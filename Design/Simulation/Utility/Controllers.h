#pragma once

#include <cmath>
#include "../Sim_Types.h"

#define MAX_THRUST 30.0
#define INDEX_NOT_SET 100

float Height_MRAC(States &MCU, float h_ref);

void Set_throttles(uint16_t motor_throttles[4], float desired_thrust, float desired_moments[3]);

void Attitude_Gain_Lookup(uint8_t count, float Gains[3][6]);

void Attitude_LQR(States &MCU, States &Reference);

void Angular_Rate_Control(States &MCU, States &Reference, float desired_moments[3]);