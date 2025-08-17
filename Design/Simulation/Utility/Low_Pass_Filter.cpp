#include "Low_Pass_Filter.h"
#include <cmath>

Low_Pass_Filter::Low_Pass_Filter(double bandwidth, double initial_value){
    Bandwidth = bandwidth;
    x = initial_value;
}

double Low_Pass_Filter::Update(double x_in, double d_t){
    // Assume that for a first order filter the bandwidth is equal to the cutoff frequency at -3dB (50% reduction in power)
    Beta = exp(-Bandwidth*d_t);
    double output = Beta*x + (1.0 - Beta)*x_in;
    x = output;
    return x;
}