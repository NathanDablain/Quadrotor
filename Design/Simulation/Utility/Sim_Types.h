#pragma once

#include <stdint.h>

#define PI 3.14159265358979311600
#define PI_2 (PI/2.0)
#define D2R (PI/180.0)
#define R2D (1.0/D2R)

struct States{
	int16_t w[3];
	double w_deg_s[3];
	double w_rads[3];
	int16_t a[3]; // In the frame Forward - Right - Down
	double m_vec[3];
	int32_t m_xyz_LSB[3];
    double Euler[3];
	double Euler_deg[3];
    double pressure_altitude;
	double Altitude_Filter_Data[2];
    double Position_NED[3];
	double Velocity_NED[3];
	double v[3];
    int32_t Longitude;
    int32_t Latitude;
    double pressure; // Temporary
    double Position_ECEF[3];
};

typedef enum{
	// Drone systems initialized, awaiting calibration
	Standby,
	// Drone systems are being calibrated by the user, involves rotating drone body
	User_Calibration,
	// Drone systems are performing automatic pre flight calibration, drone should be in position
	System_Calibration,
	// Drone systems are calibrated, ready to fly
	Ready,
	// Drone is flying, responding to commands and under autopilot control
	Flying,
	// Drone is following landing procedure, will automatically proceed to ready once complete
	Landing,
	// This type is obviously only for simulation purposes...
	Crashed
}FC_Status;

struct Uplink{
	FC_Status Drone_status;
	float Desired_north;
	float Desired_east;
	float Desired_altitude;
	float Base_altitude;
};

struct Downlink{
	FC_Status Flight_Controller_Status;
	uint8_t Tracking_Status;
	char ID[3];
};

struct Monte_Carlo_Data{
	double mass;
    double length_f_b;
    double length_l_r;
	double interia_xx;
	double inertia_yy;
	double inertia_zz;
	double inertia_xy;
	double inertia_xz;
	double inertia_yz;
    uint16_t Motor_deadzone[4];
	double Motor_zero_offset_l;
	double Motor_slope_l;
	double Motor_zero_offset_h;
	double Motor_slope_h;
	double Propeller_force_constant;
	double Propeller_torque_constant;
    double Propeller_mu;
	double Initial_roll;
	double Initial_pitch;
	double Initial_yaw;
	double Windspeed[2];
	double WindEuler[3][2];
};