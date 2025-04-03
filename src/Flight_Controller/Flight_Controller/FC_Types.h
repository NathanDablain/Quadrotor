#ifndef FC_TYPES
#define FC_TYPES

// Global variables
extern volatile unsigned long g_seconds;

#define D2R M_PI/180.0

// Structures
typedef struct{
	float w[3];
	float g_vec[3]; // In the frame Forward - Right - Down
	float m_vec[3];
	float Euler[3];
	float pressure_altitude;
	float Position_NED[3];
	signed long Longitude;
	signed long Latitude;
	float Position_ECEF[3];
	float Speed_over_ground;
	float Course_over_ground;
} States;

#endif