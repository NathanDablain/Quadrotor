#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    // data points first by row then by column
    double **data;
    uint8_t rows;
    uint8_t columns;
} Matrix;

typedef struct{
    //data points first by row then by column
    double data[3][3];
} Matrix_3;

// Variable size matrix functions

Matrix* Mat_Constructor(uint8_t rows, uint8_t columns);

void Mat_Destructor(Matrix* Mat);

Matrix* Mat_Add(Matrix *restrict Mat1, Matrix *restrict Mat2, uint8_t Destroy_flag);

Matrix* Mat_Sub(Matrix *restrict Mat1, Matrix *restrict Mat2, uint8_t Destroy_flag);

Matrix* Mat_Mul(Matrix *restrict Mat1, Matrix *restrict Mat2, uint8_t Destroy_flag);

Matrix* Mat_Tran(Matrix *restrict Mat);

Matrix* Mat_Inv(Matrix *restrict Mat);

Matrix* Mat_Id(uint8_t dim);

void Mult_and_Sub_Row(double *restrict left_side, double *restrict right_side, double con, uint8_t length);

void Multiply_row(double *restrict row, double con, uint8_t length);

void Subtract_row(double *restrict left_side, double *restrict right_side, uint8_t length);

// Fixed 3x3 size matrix functions

void Mat3_Add(Matrix_3 *restrict Mat1, Matrix_3 *restrict Mat2, Matrix_3 *restrict Res);

void Mat3_Sub(Matrix_3 *restrict Mat1, Matrix_3 *restrict Mat2, Matrix_3 *restrict Res);

void Mat3_Mul(Matrix_3 *restrict Mat1, Matrix_3 *restrict Mat2, Matrix_3 *restrict Res);

void Mat3_Tran(Matrix_3 *restrict Mat, Matrix_3 *restrict Res);

bool Mat3_Inv(Matrix_3 *restrict Mat, Matrix_3 *restrict Res);

#endif