#pragma once

#include <cstdint>

class Gaussian{
    private:
        double Variance;
        double Mean;
        double Max;
        double Min;
        uint32_t Steps;
        double *Probability;
        double *Sigma;
    public:
        Gaussian(double variance, double mean);
        ~Gaussian();
        double Get_probability(double x);
        double Get_val();
        double Get_seeded_val(uint32_t seed);
};