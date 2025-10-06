#ifndef BAROMETER_P32_H
#define BAROMETER_P32_H

#include <stdint.h>

typedef struct{
    uint8_t pressure_LSB_bytes[4];
    uint32_t pressure_LSB;
    uint8_t pressure_offset;
    double pressure_pa;
    double height;
    double height_dot;
    double base_altitude;
    double height_dot_last;
    double height_last;
} BAR_Data;

void Initialize_Bar();

void Read_Bar(uint32_t pressure_LSB, uint8_t status);

double Barometer_Altitude();

double Barometer_Altitude_Dot();

double Barometer_Pressure();

#endif