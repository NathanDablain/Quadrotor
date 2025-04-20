#ifndef FC_TYPES
#define FC_TYPES

// Global variables
extern volatile unsigned long g_seconds;

#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)

// Structures
typedef struct{
	float w[3];
	signed long w_xyz_LSB[3];
	float g_vec[3]; // In the frame Forward - Right - Down
	float m_vec[3];
	signed long m_xyz_LSB[3];
	float Euler[3];
	float pressure_altitude;
	float Position_NED[3];
	signed long Longitude;
	signed long Latitude;
	float Position_ECEF[3];
	float Speed_over_ground;
	float Course_over_ground;
} States;

typedef struct{
	// bar_cal_status -> flag with state of calibration, 0: uncalibrated, 1:ready
	// base_altitude -> offset to apply against bar reading, either from ground controller or last good reading
	unsigned char bar_cal_status;
	float base_altitude;
	// imu_cal_status -> flag with state of calibration, 0: uncalibrated, 1: ready
	// w_bias -> constant offset along each gyro axis in LSB
	unsigned char imu_cal_status;
	signed int w_bias[3];
	// Magnetometer cal data
	// mag_cal_status -> flag with state of calibration, 0: uncalibrated, 1: partial calibration, 2: ready
	// m_max -> max magnetic field recorded along each axis
	// m_min -> min magnetic field recorded along each axis
	// hard_iron -> offset incurred by nearby hard iron sources, shifts local field off 0 mean
	unsigned char mag_cal_status;
	signed int m_max[3];
	signed int m_min[3];
	signed int hard_iron[3];
	// GPS cal data
	// gps_cal_status -> flag with state of calibration, 0: uncalibrated, 1: ready
	// Reference_Position_ecef -> Earth Centered Earth Fixed coordinates of initial reference position which NED offset is based on
	unsigned char gps_cal_status;
	float Reference_Position_ecef[3];
} Calibration_Data;

void Delay(unsigned long long length);


#endif