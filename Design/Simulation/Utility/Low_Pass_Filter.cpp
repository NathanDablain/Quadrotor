#include "Low_Pass_Filter.h"
#include <cmath>

Low_Pass_Filter::Low_Pass_Filter(){}

Low_Pass_Filter::Low_Pass_Filter(double bandwidth, double initial_value){
    Initialize(bandwidth, initial_value);
}

void Low_Pass_Filter::Initialize(double bandwidth, double initial_value){
    Bandwidth = bandwidth;
    x = initial_value;
}

double Low_Pass_Filter::Update(double x_in, double d_t, bool passthrough){
    // Assume that for a first order filter the bandwidth is equal to the cutoff frequency at -3dB (50% reduction in power)
    Beta = exp(-Bandwidth*d_t);
    double output = passthrough?x_in:Beta*x + (1.0 - Beta)*x_in;
    x = output;
    return x;
}

void Low_Pass_Filter::Change_Bandwidth(double new_bandwidth){
    Bandwidth = new_bandwidth;
}
