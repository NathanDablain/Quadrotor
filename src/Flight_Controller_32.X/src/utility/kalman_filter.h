#ifndef KALMAN_FILTER_H
#define	KALMAN_FILTER_H

#include "linear_algebra.h"
#include "time.h"
#include <stdint.h>
#include <stdbool.h>

#define FULL_FILTER_DESTRUCTOR 6

typedef struct {
    Matrix *F;
    Matrix *H;
    Matrix *B;
    Matrix *Q;
    Matrix *R;
    Matrix *P;
    Matrix *K;
    Matrix *xhat;
} Kalman_Filter;

Kalman_Filter* Filter_Constructor(uint8_t states, uint8_t inputs, uint8_t measurements);

void Filter_Destructor(Kalman_Filter *filter, uint8_t stage);

bool Predict(Kalman_Filter *filter, Matrix *input);

bool Update(Kalman_Filter *filter, Matrix *measurement);

#endif	
