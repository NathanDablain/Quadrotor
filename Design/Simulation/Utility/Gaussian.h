#pragma once

#include <cstdint>

class Gaussian{
    private:
        double Variance;
        double Mean;
        double Max;
        double Min;
        double Probability[500];
        double Sigma[500];
    public:
        Gaussian();
        Gaussian(double variance, double mean);
        void Initialize(double variance, double mean);
        double Get_probability(double x);
        double Get_val();
        double Get_seeded_val(uint32_t seed);
};