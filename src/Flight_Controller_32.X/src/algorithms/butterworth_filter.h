#ifndef BUTTERWORTH_FILTER_P32_H
#define BUTTERWORTH_FILTER_P32_H

typedef struct{
    double w_c;
    double x[5];
    double xdot_prev[4];
    double d_t;
    double u;
} BW_Filter_Data;

void Initialize_BW_Filter(BW_Filter_Data* Filter, double d_t);

double Run_BW_Filter(BW_Filter_Data* Filter);

#endif

