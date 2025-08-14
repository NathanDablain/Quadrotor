#pragma once

#include <stdint.h>
#include <stdint.h>
#include <stdbool.h>

#define US_IN_S 1000000

extern volatile uint32_t g_seconds;
extern volatile uint32_t g_microseconds;

typedef struct{
    int32_t seconds;
    int32_t microseconds;
} Time;

typedef struct{
    Time imu_clk;
    Time mag_clk;
    Time bar_clk;
    Time lora_clk;
    Time gps_clk;
} SEQUENCER;

Time Current_Time();

Time Time_Difference(Time Time_1, Time Time_2);

double Time_fp(Time time);

bool Compare_And_Update(const Time Time_1, Time* Time_2);

typedef struct{
    // data points first by row then by column
    float **data;
    uint8_t rows;
    uint8_t columns;
} Matrix;

Matrix* Mat_Constructor(uint8_t rows, uint8_t columns);

void Mat_Destructor(Matrix* Mat);

Matrix* Mat_Add(Matrix* Mat1, Matrix* Mat2, uint8_t Destroy_flag);

Matrix* Mat_Sub(Matrix* Mat1, Matrix* Mat2, uint8_t Destroy_flag);

Matrix* Mat_Mul(Matrix* Mat1, Matrix* Mat2, uint8_t Destroy_flag);

Matrix* Mat_Tran(Matrix* Mat);

Matrix* Mat_Inv(Matrix* Mat);

Matrix* Mat_Id(uint8_t dim);

void Mult_and_Sub_Row(float *left_side, float *right_side, float con, uint8_t length);

void Multiply_row(float *row, float con, uint8_t length);

void Subtract_row(float *left_side, float *right_side, uint8_t length);

typedef struct {
    Matrix *F;
    Matrix *H;
    Matrix *B;
    Matrix *Q;
    Matrix *R;
    Matrix *P;
    Matrix *xhat;
    Time Last_prediction;
    Time Last_update;
} Kalman_Filter;

Kalman_Filter* Filter_Constructor(uint8_t states, uint8_t inputs, uint8_t measurements);

void Filter_Destructor(Kalman_Filter *filter, uint8_t stage);

bool Predict(Kalman_Filter *filter, Matrix *input);

bool Update(Kalman_Filter *filter, Matrix *measurement);

void Safe_Free(void *ptr);

class PIC32AK{
    private:
    public:
        PIC32AK();
};