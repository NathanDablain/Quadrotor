#pragma once

#include <cstdint>

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