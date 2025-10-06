#ifndef CONTROLLERS_P32_H
#define CONTROLLERS_P32_H

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    double Thrust;
    double Hover_Thrust;
    double Moments[3];
    uint16_t Throttles[4];
    double Thrust_e_int;
    double Mass_Modifier;
    double Moment_e_int[3];
} Control_Variables;

void Initialize_Controllers();

void Thrust_Control();

void Moment_Control();

void Set_throttles();

void Saturate(double *Value, double Min, double Max);

#endif