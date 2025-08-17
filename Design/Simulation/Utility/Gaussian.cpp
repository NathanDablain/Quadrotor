#include "Gaussian.h"
#include "cstdint"
#include "cmath"
#include <iostream>

Gaussian::Gaussian(double variance, double mean){
    Variance = variance;
    Mean = mean;
    Max = 6.0*Variance + Mean;
    Steps = 200;
    // Start at mean and move outwards to the max, integrate area under curve and store in array for lookup
    double step = (Max-Mean)/static_cast<double>(Steps);
    Probability = new double[Steps];
    Sigma = new double[Steps];
    double x = Mean;
    double sum = 0.0;
    for (uint32_t i = 0; i < Steps; i++){
        sum += Get_probability(x)*step;
        Probability[i] = 2.0*sum;
        Sigma[i] = (Mean - x);
        x += step;
    }
}

Gaussian::~Gaussian(){
    delete[] Probability;
    delete[] Sigma;
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
    for (uint32_t i = 0; i < Steps; i++){
        if (Probability[i] > random_number){
            return (sign > RAND_MAX/2)?(Mean + Sigma[i]):(Mean - Sigma[i]);
        }
    }


    return Sigma[0];
}

double Gaussian::Get_seeded_val(uint32_t seed){
    double seeded_val;
    for (uint32_t i = 0; i < seed; i ++){
        seeded_val = Get_val();
    }
    return seeded_val;
}