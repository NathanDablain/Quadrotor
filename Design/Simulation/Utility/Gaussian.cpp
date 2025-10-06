#include "Gaussian.h"
#include "cstdint"
#include "cmath"
#include <iostream>

static uint32_t Steps = 500;

Gaussian::Gaussian(){}

Gaussian::Gaussian(double variance, double mean){
    Initialize(variance, mean);
}

void Gaussian::Initialize(double variance, double mean){
    Variance = variance;
    Mean = mean;
    Max = 6.0*Variance + Mean;
    // Start at max and move inwards to the mean, integrate area under curve and store in array for lookup
    double step = (Mean-Max)/static_cast<double>(Steps);
    double x = Max;
    double sum = 0.0;
    for (uint32_t i = 0; i < Steps; i++){
        sum += Get_probability(x)*fabs(step);
        Probability[i] = 2.0*sum;
        Sigma[i] = (Mean - x);
        x += step;
    }
}

double Gaussian::Get_probability(double x){
    double pi = 3.14159;
    double probability = (1.0/(Variance*sqrt(2.0*pi)))*exp(-(pow(x - Mean, 2)/(2.0*Variance*Variance)));

    return probability;
}

double Gaussian::Get_val(){
    // Ensure that random_number is between 0 and 1
    double random_number = static_cast<double>(rand())/RAND_MAX;
    uint32_t sign = rand();
    for (int_fast16_t i = Steps-1; i >= 0; i--){
        if (Probability[i] < random_number){
            return (sign > RAND_MAX/2)?(Mean + Sigma[i]):(Mean - Sigma[i]);
        }
    }


    return Sigma[0];
}

double Gaussian::Get_seeded_val(uint32_t seed){
    double seeded_val = 0.0;
    for (uint32_t i = 0; i < seed; i ++){
        seeded_val = Get_val();
    }
    return seeded_val;
}