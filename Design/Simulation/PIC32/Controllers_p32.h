#ifndef CONTROLLERS_P32_H
#define CONTROLLERS_P32_H

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    double Thrust;
    double Moments[3];
    uint16_t Throttles[4];
} Control_Variables;

void Initialize_Controllers();

void Run_Controllers();

void Thrust_Control();

void Moment_Control();

void Set_throttles();

void Saturate(double *Value, double Min, double Max);

#endif