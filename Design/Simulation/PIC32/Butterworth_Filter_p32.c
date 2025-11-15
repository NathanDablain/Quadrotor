#include <string.h>
#include <math.h>
#include "Butterworth_Filter_p32.h"

void Initialize_BW_Filter(BW_Filter_Data* Filter, double d_t){
    memset(Filter, 0, sizeof(BW_Filter_Data));
    Filter->d_t = d_t;
}

double Run_BW_Filter(BW_Filter_Data* Filter){
    const double coef[5] = {1.0, 2.6131, 3.4142, 2.6131, 1.0};
    double modified_coef[5];
    for (uint8_t i = 0; i < 5; i++){
        modified_coef[i] = coef[i] / pow(Filter->w_c, 4 - i);
    }
    // divide coefficients by filter cutoff frequency to obtain modified coefficients
    // a0s4 + a1s3 + a2s2 + a3s + a4 = u -> x4dot = 1/a0 * (u - a1s3 - a2s2 - a3s - a4)
    /*  x4dot = ...
        x3dot = x4 = x4_prev + 0.5*dt*(x4dot + x4dot_prev)
        x2dot = x3
        x1dot = x2
        x1 = x1 + ...x1dot
    */
    Filter->x[4] = (1.0 / modified_coef[0])*(Filter->u - modified_coef[1]*Filter->x[3] - modified_coef[2]*Filter->x[2]
                    - modified_coef[3]*Filter->x[1] - modified_coef[4]*Filter->x[0]);
    Filter->x[3] += 0.5 * Filter->d_t * (Filter->x[4] + Filter->xdot_prev[3]);
    Filter->x[2] += 0.5 * Filter->d_t * (Filter->xdot_prev[3] + Filter->xdot_prev[2]);
    Filter->x[1] += 0.5 * Filter->d_t * (Filter->xdot_prev[2] + Filter->xdot_prev[1]);
    Filter->x[0] += 0.5 * Filter->d_t * (Filter->xdot_prev[1] + Filter->xdot_prev[0]);
    memcpy(&Filter->xdot_prev[0], &Filter->x[1], 4 * sizeof(double));

    return Filter->x[0];
}