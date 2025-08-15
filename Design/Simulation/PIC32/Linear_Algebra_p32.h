#ifndef LINEAR_ALGEBRA_P32_H
#define LINEAR_ALGEBRA_P32_H

#include <stdint.h>
#include <stdbool.h>

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

#endif