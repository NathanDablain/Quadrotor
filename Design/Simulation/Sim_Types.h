#pragma once

#include <cstdint>

#define PI 3.14159265358979311600
#define PI_2 PI/2.0
#define D2R PI/180.0
#define R2D 1.0/D2R

struct States{
    float w[3];
    float Euler[3];
    float g_vec[3];
    float m_vec[3];
    float pressure_altitude;
    float Position_NED[3];
    int32_t Longitude;
    int32_t Latitude;
    float pressure; // Temporary
    float Position_ECEF[3];
};