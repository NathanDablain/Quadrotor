#ifndef KALMAN_FILTER_P32_H
#define KALMAN_FILTER_P32_H

#include "Linear_Algebra_p32.h"
#include "Time_p32.h"
#include <stdint.h>


typedef struct {
    Matrix *F;
    Matrix *H;
    Matrix *B;
    Matrix *Q;
    Matrix *R;
    Matrix *P;
    Matrix *xhat;
    Time Last_prediction;
    Time Last_update;
} Kalman_Filter;

Kalman_Filter* Filter_Constructor(uint8_t states, uint8_t inputs, uint8_t measurements);

void Filter_Destructor(Kalman_Filter *filter, uint8_t stage);

bool Predict(Kalman_Filter *filter, Matrix *input);

bool Update(Kalman_Filter *filter, Matrix *measurement);

#endif