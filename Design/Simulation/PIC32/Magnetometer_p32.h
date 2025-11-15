#ifndef MAGNETOMETER_P32_H
#define MAGNETOMETER_P32_H

#include <stdint.h>
#include "Time_p32.h"
#include "Linear_Algebra_p32.h"

#define MAG_SENSITIVITY 1.5

typedef enum{
    Mag_Standby,
    Mag_Fail,
    Mag_Ready,
    Mag_Reading
} Mag_Machine;

typedef struct{
    double field[3];
    double field_filtered[3];
    uint8_t field_LSB_bytes[7];
    int16_t field_LSB[3];
    int16_t hard_iron[3];
    int16_t mag_values_at_max[3][3];
    int16_t mag_field_max_LSB[3];
    int16_t mag_field_min_LSB[3];
    Matrix_3 soft_iron;
    Time Last_Update;
    bool offset_initialized[3];
} Mag_Data;

void Initialize_Mag();

void Run_Magnetometer_Machine();

void Convert_Magnetometer();

bool Calculate_Hard_Iron();

bool Calculate_Soft_Iron();

void Magnetometer_LPF(uint8_t setting);

void Compensate_Magnetometer_Reading(bool Soft_iron_cal);

double Magnetometer_Field(uint8_t index);

double Magnetometer_Filtered_Field(uint8_t index);

#endif