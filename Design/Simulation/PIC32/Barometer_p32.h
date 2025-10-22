#ifndef BAROMETER_P32_H
#define BAROMETER_P32_H

#include "Time_p32.h"
#include <stdint.h>

typedef enum{
    BAR_Standby,
    BAR_Fail,
    BAR_Ready,
    BAR_Reading
} BAR_Machine;

typedef struct{
    uint8_t pressure_LSB_bytes[4];
    uint32_t pressure_LSB;
    uint8_t pressure_offset;
    double pressure_pa;
    double height;
    double base_altitude;
    Time Last_Update;
} BAR_Data;

void Initialize_Bar();

void Run_Barometer_Machine();

void Convert_Pressure();

double Barometer_Altitude();

double Barometer_Pressure();

#endif