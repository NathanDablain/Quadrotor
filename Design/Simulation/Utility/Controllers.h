#pragma once

#include <cmath>
#include <cstring>
#include "../Sim_Types.h"

#define MOTOR_CUTOFF_ANGLE (45.0*D2R)
#define MOTOR_CUTOFF_ALTITUDE 7.0f
#define INDEX_NOT_SET 100

struct Drone_Constants{
	// Propeller thrust constant N/(rad/s)^2
	float k_f;
	// Propeller torque constant N-m/(rad/s)^2
	float k_t;
	// Distance from front and back motor thrust vectors to drone center of gravity (m)
	float length_f_b;
	// Distance from left and right motor thrust vectors to drone center of gravity (m)
	float length_l_r;
	// Mass moments of inertia
	float I[3];
	// Mass (kg)
	float mass;
	// Motor torque constant in N-m/A (1/KV)
	float KT;
	// Gain to convert (rad/s)^2 to A
	float K;
	// The current produced by the motor for each 10% of throttle, starting at 0%
	float Current[11];
};

Drone_Constants Initialize_Drone_Constants();

float Altitude_Control(float h, float h_ref, const Drone_Constants *Constants);

void Set_throttles(uint16_t motor_throttles[4], float desired_thrust, float desired_moments[3], const Drone_Constants *Constants);

void Euler_Control(float Current_Euler[3], float Commanded_Euler[3], float desired_moments[3], float thrust, const Drone_Constants *Constants);

void Saturate(float &desired_moment, float max, float min, uint8_t &Saturation_Flag);

void Safety_Check(uint16_t motor_throttles[4], States *Drone, FC_Status *Flight_Controller_Status);

void Run_Guidance(Reference *Desired_States, Reference *Commanded_States);