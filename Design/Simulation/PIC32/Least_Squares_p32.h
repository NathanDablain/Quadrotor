#ifndef LEAST_SQUARES_P32_H
#define LEAST_SQUARES_P32_H

#include <stdint.h>
#include <stdbool.h>

#define LEAST_SQUARES_SAMPLE_SIZE 20

typedef struct{
    double Sample_Points_IV[LEAST_SQUARES_SAMPLE_SIZE];
    double Sample_Points_DV[LEAST_SQUARES_SAMPLE_SIZE];
    uint8_t Sample_Counter;
    bool Slope_Calculated;
} Least_Squares_Dataset;

typedef struct{
    double slope;
    double intercept;
} Least_Squares_Line;

Least_Squares_Line Least_Squares_Slope(Least_Squares_Dataset *data);

double Least_Squares_Point(Least_Squares_Line *line, double iv);

#endif