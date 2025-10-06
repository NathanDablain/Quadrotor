#ifndef NAVIGATION_H
#define	NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct{
    // Diagonal terms of covariance matrix are variances
    double P1_1;
    double P2_2;
    double P3_3;
    double P4_4;
    double P5_5;
    double P6_6;
    double P7_7;
    double P8_8;
    double P9_9;
    // Off diagonal terms are covariances
    double P1_2;
    double P1_3;
    double P1_4;
    double P1_5;
    double P1_6;
    double P1_7;
    double P1_8;
    double P1_9;

    double P2_3;
    double P2_4;
    double P2_5;
    double P2_6;
    double P2_7;
    double P2_8;
    double P2_9;

    double P3_4;
    double P3_5;
    double P3_6;
    double P3_7;
    double P3_8;
    double P3_9;

    double P4_5;
    double P4_6;
    double P4_7;
    double P4_8;
    double P4_9;

    double P5_6;
    double P5_7;
    double P5_8;
    double P5_9;

    double P6_7;
    double P6_8;
    double P6_9;

    double P7_8;
    double P7_9;

    double P8_9;
} Ground_Filter_Covariance;

typedef struct{
    // Diagonal terms of covariance matrix are variances
    double P1_1;
    double P2_2;
    double P3_3;
    double P4_4;
    double P5_5;
    double P6_6;
    // Off diagonal terms are covariances
    double P1_2;
    double P1_3;
    double P1_4;
    double P1_5;
    double P1_6;

    double P2_3;
    double P2_4;
    double P2_5;
    double P2_6;

    double P3_4;
    double P3_5;
    double P3_6;

    double P4_5;
    double P4_6;

    double P5_6;
} Air_Filter_Covariance;

void Run_Ground_Filter(bool Initialize);

void Initialize_Ground_Filter();

void Ground_Filter_Predict();

void Ground_Filter_Update();

double Ground_Filter_data(uint8_t index);

void Run_Air_Filter(bool Initialize);

void Initialize_Air_Filter();

void Air_Filter_Predict();

void Air_Filter_Update();

#endif

