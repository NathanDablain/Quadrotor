#pragma once

#include <cmath>
#include "../Sim_Types.h"

#define MAX_THRUST 10.0 // N
#define MAX_MOMENT 0.000002 // N-m
#define MAX_W 2.0 // rad/s
#define INDEX_NOT_SET 100

float Height_LQR(float h, float h_ref);

void Set_throttles(uint16_t motor_throttles[4], float desired_thrust, float desired_moments[3]);

void Angular_Rate_Control(States &MCU, States &Reference, float desired_moments[3]);