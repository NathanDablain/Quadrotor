#include "Least_Squares_p32.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

Least_Squares_Line Least_Squares_Slope(Least_Squares_Dataset *data){
    // Least_Squares_Line result;
    // double IV_avg = 0.0;
    // double DV_avg = 0.0;
    // for (uint8_t i = 0; i < LEAST_SQUARES_SAMPLE_SIZE; i++){
    //     IV_avg += data->Sample_Points_IV[i];
    //     DV_avg += data->Sample_Points_DV[i];
    // }
    // IV_avg /= (double)LEAST_SQUARES_SAMPLE_SIZE;
    // DV_avg /= (double)LEAST_SQUARES_SAMPLE_SIZE;

    // double IV_diff = 0.0;
    // double DV_diff = 0.0;
    // double numerator = 0.0;
    // double denominator = 0.0;
    // for (uint8_t i = 0; i < LEAST_SQUARES_SAMPLE_SIZE; i++){
    //     IV_diff = IV_avg - data->Sample_Points_IV[i];
    //     DV_diff = DV_avg - data->Sample_Points_DV[i];
    //     numerator += (IV_diff * DV_diff);
    //     denominator += pow(IV_diff, 2);
    // }
    Least_Squares_Line result;
    double IV_sum = 0.0;
    double DV_sum = 0.0;
    double IV_DV = 0.0;
    double DV_2 = 0.0;
    for (uint8_t i = 0; i < LEAST_SQUARES_SAMPLE_SIZE; i++){
        IV_sum += data->Sample_Points_IV[i];
        DV_sum += data->Sample_Points_DV[i];
        IV_DV += (data->Sample_Points_IV[i] * data->Sample_Points_DV[i]);
        DV_2 += pow(data->Sample_Points_DV[i], 2);
    }
    double N = (double)LEAST_SQUARES_SAMPLE_SIZE;
    double numerator = (N*IV_DV) - (DV_sum * IV_sum);
    double denominator = (N*DV_2) - pow(DV_sum, 2);

    result.slope = numerator / denominator;
    result.intercept = (DV_sum - result.slope*IV_sum)/N;
    data->Slope_Calculated = true;
    return result; 

}

double Least_Squares_Point(Least_Squares_Line *line, double iv){
    double result = line->intercept + line->slope*iv;
    return result;
}
