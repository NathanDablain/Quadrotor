#pragma once

#include <cmath>
#include "../Sim_Types.h"

#define MAX_THRUST 30.0

float Height_MRAC(States &MCU, float h_ref);

void Set_throttles(uint16_t motor_throttles[4], float desired_thrust, float desired_moments[3]);