#ifndef MAGNETOMETER_P32_H
#define MAGNETOMETER_P32_H

#include <stdint.h>
#include "Linear_Algebra_p32.h"

#define MAG_SENSITIVITY 1.5

typedef struct{
    double field[3];
    uint8_t field_LSB_bytes[7];
    int16_t field_LSB[3];
    int16_t hard_iron[3];
    int16_t mag_values_at_max[3][3];
    int16_t mag_field_max_LSB[3];
    int16_t mag_field_min_LSB[3];
    Matrix_3 soft_iron;
} Mag_Data;

void Initialize_Mag();

void Read_Mag(int16_t magnetic_field[3]);

bool Calculate_Hard_Iron();

bool Calculate_Soft_Iron();

void Compensate_Magnetometer_Reading(bool Soft_iron_cal);

double Magnetometer_Field(uint8_t index);

#endif