#pragma once

#include <cstdint>

#define PI 3.14159265358979311600
#define PI_2 (PI/2.0)
#define D2R (PI/180.0)
#define R2D (1.0/D2R)

struct States{
	int16_t w[3];
	int16_t g_vec[3]; // In the frame Forward - Right - Down
	float m_vec[3];
	int32_t m_xyz_LSB[3];
    float Euler[3];
    float pressure_altitude;
    float Position_NED[3];
    int32_t Longitude;
    int32_t Latitude;
    float pressure; // Temporary
    float Position_ECEF[3];
};

struct Calibration_Data{
	// bar_cal_status -> flag with state of calibration, 0: uncalibrated, 1:ready
	// altitude_bias -> offset to apply against bar reading, either from ground controller or last good reading
	uint8_t bar_cal_status;
	float altitude_bias;
	// imu_cal_status -> flag with state of calibration, 0: uncalibrated, 1: ready
	// w_bias -> constant offset along each gyro axis in LSB
	uint8_t imu_cal_status;
	int16_t w_bias[3];
	// Magnetometer cal data
	// mag_cal_status -> flag with state of calibration, 0: uncalibrated, 1: partial calibration, 2: ready
	// m_max -> max magnetic field recorded along each axis
	// m_min -> min magnetic field recorded along each axis
	// hard_iron -> offset incurred by nearby hard iron sources, shifts local field off 0 mean
	uint8_t mag_cal_status;
	int16_t m_max[3];
	int16_t m_min[3];
	int16_t hard_iron[3];
	// GPS cal data
	// gps_cal_status -> flag with state of calibration, 0: uncalibrated, 1: ready
	// Reference_Position_ecef -> Earth Centered Earth Fixed coordinates of initial reference position which NED offset is based on
	uint8_t gps_cal_status;
	float Reference_Position_ecef[3];
};

enum FC_Status{
	// Drone systems initialized, awaiting calibration
	Standby,
	// Drone systems calibrating
	Calibrating,
	// Drone systems are calibrated, ready to fly
	Ready,
	// Drone is flying, responding to commands and under autopilot control
	Flying,
	// Drone is following landing procedure, will automatically proceed to ready once complete
	Landing
};

struct Uplink{
	FC_Status Drone_status;
	float Desired_north;
	float Desired_east;
	float Desired_altitude;
	float Base_altitude;
};

struct Downlink{
	// Are we calibrated
	uint8_t Calibration_Status;
	// Are we tracking the reference well
	uint8_t Tracking_Status;
	char ID[3];
};