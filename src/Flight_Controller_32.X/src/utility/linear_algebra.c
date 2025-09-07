#include "linear_algebra.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

Matrix* Mat_Constructor(uint8_t rows, uint8_t columns){
    Matrix* Mat_new = (Matrix *)malloc(sizeof(Matrix));
    if (Mat_new == NULL) return NULL;

    Mat_new->data = (double **)malloc(rows*sizeof(double*));
    Mat_new->rows = rows;
    if (Mat_new->data == NULL){
        free(Mat_new);
        return NULL;
    }
    for (uint_fast8_t i = 0; i != rows; i++){
        Mat_new->data[i] = (double *)calloc(columns, sizeof(double));
        if (Mat_new->data[i] == NULL){
            if (i > 0){
                for (int_fast16_t j = i-1; j >= 0; j--){
                    free(Mat_new->data[j]);
                }
            }
            free(Mat_new->data);
            free(Mat_new);
            return NULL;
        }
    }
    Mat_new->columns = columns;

    return Mat_new;
}

void Mat_Destructor(Matrix* Mat){
    for (uint_fast8_t i = 0; i != Mat->rows; i++){
        free(Mat->data[i]);
    }
    free(Mat->data);
    free(Mat);
}

Matrix* Mat_Add(Matrix *restrict Mat1, Matrix *restrict Mat2, uint8_t Destroy_flag){
    // Ensure matrices are the same dimension
    if ((Mat1->columns != Mat2->columns) || (Mat1->rows != Mat2->rows)) return NULL;
    // Build resultant matrix
    Matrix* Mat_result = Mat_Constructor(Mat1->rows, Mat2->columns);
    if (Mat_result == NULL) return NULL;

    for (uint_fast8_t i = 0; i != Mat1->rows; i++){
        for (uint_fast8_t j = 0; j != Mat1->columns; j++){
            Mat_result->data[i][j] = Mat1->data[i][j] + Mat2->data[i][j];
        }
    }

    if (Destroy_flag == 1){
        Mat_Destructor(Mat1);
    }
    else if (Destroy_flag == 2){
        Mat_Destructor(Mat2);
    }
    else if (Destroy_flag == 3){
        Mat_Destructor(Mat1);
        Mat_Destructor(Mat2);
    }
    
    return Mat_result;
}

Matrix* Mat_Sub(Matrix *restrict Mat1, Matrix *restrict Mat2, uint8_t Destroy_flag){
    // Ensure matrices are the same dimension
    if ((Mat1->columns != Mat2->columns) || (Mat1->rows != Mat2->rows)) return NULL;
    // Build resultant matrix
    Matrix* Mat_result = Mat_Constructor(Mat1->rows, Mat2->columns);
    if (Mat_result == NULL) return NULL;

    for (uint_fast8_t i = 0; i != Mat1->rows; i++){
        for (uint_fast8_t j = 0; j != Mat1->columns; j++){
            Mat_result->data[i][j] = Mat1->data[i][j] - Mat2->data[i][j];
        }
    }
    
    if (Destroy_flag == 1){
        Mat_Destructor(Mat1);
    }
    else if (Destroy_flag == 2){
        Mat_Destructor(Mat2);
    }
    else if (Destroy_flag == 3){
        Mat_Destructor(Mat1);
        Mat_Destructor(Mat2);
    }
    
    return Mat_result;
}

Matrix* Mat_Mul(Matrix *restrict Mat1, Matrix *restrict Mat2, uint8_t Destroy_flag){
    // Ensure matrix 1 has the same number of columns as matrix 2 has rows
    if (Mat1->columns != Mat2->rows) return NULL;
    // Build resultant matrix
    Matrix* Mat_result = Mat_Constructor(Mat1->rows, Mat2->columns);
    if (Mat_result == NULL) return NULL;

    for (uint_fast8_t i = 0; i != Mat1->rows; i++){
        for (uint_fast8_t j = 0; j != Mat2->columns; j++){
            for (uint_fast8_t k = 0; k != Mat1->columns; k++){
                double temp1 = Mat1->data[i][k];
                double temp2 = Mat2->data[k][j];
                Mat_result->data[i][j] += temp1 * temp2;
                // Mat_result->data[i][j] += Mat1->data[i][k] * Mat2->data[k][j];
            }
        }
    }
    
    if (Destroy_flag == 1){
        Mat_Destructor(Mat1);
    }
    else if (Destroy_flag == 2){
        Mat_Destructor(Mat2);
    }
    else if (Destroy_flag == 3){
        Mat_Destructor(Mat1);
        Mat_Destructor(Mat2);
    }
    
    return Mat_result;
}

Matrix* Mat_Tran(Matrix *restrict Mat){
    // Build resultant matrix
    Matrix* Mat_result = Mat_Constructor(Mat->columns, Mat->rows);
    if (Mat_result == NULL) return NULL;

    for (uint_fast8_t i = 0; i != Mat->columns; i++){
        for (uint_fast8_t j = 0; j != Mat->rows; j++){
            Mat_result->data[i][j] = Mat->data[j][i];
        }
    }

    return Mat_result;
}

Matrix* Mat_Inv(Matrix *restrict Mat){
    // Ensure matrix has the same number of columns as rows
    if (Mat->columns != Mat->rows) return NULL;
    // Mat temp will be used to hold the values of the input Mat for this functions manipulation
    Matrix Mat_temp = *Mat;
    // Mat result will begin as the identity matrix and become the inverse of Mat through elementary row operations
    Matrix* Mat_result = Mat_Id(Mat->rows);
    if (Mat_result == NULL) return NULL;

    uint8_t n = Mat->rows;
    const double Mat_inv_threshold = 1e-3;

    // Use gauss-jordan elimination to solve system of linear equations
    // First work along rows until you get row echelon form, then work backwards to get reduced row echelon form
    // Search along row until you get to the diagonal entry, if any values before this entry are not zero, subtract their value*
    
    // Mat_result will be on the right hand side and Mat_temp will be on the left hand side
    for (uint_fast8_t i = 0; i != n; i++){ // Iterate down rows
        if (Mat_temp.data[i][i] == 0.0){
            for (uint_fast8_t j = 0; j != n; j++){
                if (Mat_temp.data[j][i] != 0.0){
                    Subtract_row(Mat_temp.data[i], Mat_temp.data[j], n);
                    Subtract_row(Mat_result->data[i], Mat_result->data[j], n);
                    break;
                }
            }
        }
        for (uint_fast8_t j = 0; j != i; j++){ // Iterate along row length
            if ((Mat_temp.data[i][j] != 0) && (Mat_temp.data[j][j] == 1)){
                // subtract from both sides row j multiplied by left hand matrix [i][j]
                double temp[n];
                double temp_con = Mat_temp.data[i][j];
                memcpy(temp, Mat_result->data[j], n*sizeof(double));
                for (uint_fast8_t k = 0; k != n; k++){Mat_result->data[i][k] -= (temp[k]*temp_con);}
                memcpy(temp, Mat_temp.data[j], n*sizeof(double));
                for (uint_fast8_t k = 0; k != n; k++){Mat_temp.data[i][k] -= (temp[k]*temp_con);}
            }
        }
        if ((Mat_temp.data[i][i] != 1)&&(Mat_temp.data[i][i] != 0)){
            Multiply_row(Mat_result->data[i], (1.0/Mat_temp.data[i][i]), n);
            Multiply_row(Mat_temp.data[i], (1.0/Mat_temp.data[i][i]), n);
        }
    }
    // Now work backwards to convert matrix to reduced row echelon form 
    for (int_fast16_t i = n - 1; i >= 0; i--){
        for (uint_fast8_t j = n - 1; j > i; j--){
            if ((Mat_temp.data[i][j] != 0) && (Mat_temp.data[j][j] == 1)){
                // subtract from both sides row j multiplied by left hand matrix [i][j]
                double temp[n];
                double temp_con = Mat_temp.data[i][j];
                memcpy(temp, Mat_result->data[j], n*sizeof(double));
                for (uint_fast8_t k = 0; k != n; k++){Mat_result->data[i][k] -= (temp[k]*temp_con);}
                memcpy(temp, Mat_temp.data[j], n*sizeof(double));
                for (uint_fast8_t k = 0; k != n; k++){Mat_temp.data[i][k] -= (temp[k]*temp_con);}
            }
        }
    }
    
    // Double check result, Mat_temp should now be an identity matrix
    for (uint_fast8_t i = 0; i != n; i++){
        for (uint_fast8_t j = 0; j != n; j++){
            double check_val = (i == j)?(fabs(Mat_temp.data[i][j] - 1.0)):fabs(Mat_temp.data[i][j]);
            if (check_val > Mat_inv_threshold){
                return NULL;
            }
        }
    }

    return Mat_result;
}

Matrix* Mat_Id(uint8_t dim){
    Matrix* Mat_result = Mat_Constructor(dim, dim);
    if (Mat_result == NULL) return NULL;

    for (uint_fast8_t i = 0; i != dim; i++){
        Mat_result->data[i][i] = 1.0;
    }

    return Mat_result;
}

void Mult_and_Sub_Row(double *restrict left_side, double *restrict right_side, double con, uint8_t length){
    for (uint_fast8_t i = 0; i != length; i++){
        left_side[i] -= right_side[i]*con; 
    }
}

void Multiply_row(double *restrict row, double con, uint8_t length){
    for (uint_fast8_t i = 0; i != length; i++){
        row[i] *= con;
    }
}

void Subtract_row(double *restrict left_side, double *restrict right_side, uint8_t length){
    for (uint_fast8_t i = 0; i != length; i++){
        left_side[i] -= right_side[i];
    }
}

// Fixed 3x3 size matrix functions

void Mat3_Add(Matrix_3 *restrict Mat1, Matrix_3 *restrict Mat2, Matrix_3 *restrict Res){
    for (uint_fast8_t i = 0; i < 3; i++){
        for (uint_fast8_t j = 0; j < 3; j++){
            Res->data[i][j] = Mat1->data[i][j] + Mat2->data[i][j];
        }
    }
}

void Mat3_Sub(Matrix_3 *restrict Mat1, Matrix_3 *restrict Mat2, Matrix_3 *restrict Res){
    for (uint_fast8_t i = 0; i < 3; i++){
        for (uint_fast8_t j = 0; j < 3; j++){
            Res->data[i][j] = Mat1->data[i][j] - Mat2->data[i][j];
        }
    }
}

void Mat3_Mul(Matrix_3 *restrict Mat1, Matrix_3 *restrict Mat2, Matrix_3 *restrict Res){
    for (uint_fast8_t i = 0; i < 3; i++){
        for (uint_fast8_t j = 0; j < 3; j++){
            double temp1 = Mat1->data[i][0] * Mat2->data[0][j];
            double temp2 = Mat1->data[i][1] * Mat2->data[1][j];
            double temp3 = Mat1->data[i][2] * Mat2->data[2][j];
            Res->data[i][j] = temp1 + temp2 + temp3;
        }
    }
}

void Mat3_Tran(Matrix_3 *restrict Mat, Matrix_3 *restrict Res){
    for (uint_fast8_t i = 0; i < 3; i++){
        for (uint_fast8_t j = 0; j < 3; j++){
            Res->data[i][j] = Mat->data[j][i];
        }
    }
}

bool Mat3_Inv(Matrix_3 *restrict Mat, Matrix_3 *restrict Res){
    Matrix_3 Mat_temp = *Mat;
    Res->data[0][0] = 1.0;
    Res->data[1][1] = 1.0;
    Res->data[2][2] = 1.0;
    const uint8_t n = 3;
    const double Mat_inv_threshold = 1e-3;

    // Use gauss-jordan elimination to solve system of linear equations
    // First work along rows until you get row echelon form, then work backwards to get reduced row echelon form
    // Search along row until you get to the diagonal entry, if any values before this entry are not zero, subtract their value*
    
    // Mat_result will be on the right hand side and Mat_temp will be on the left hand side
    for (uint_fast8_t i = 0; i != n; i++){ // Iterate down rows
        if (Mat_temp.data[i][i] == 0.0){
            for (uint_fast8_t j = 0; j != n; j++){
                if (Mat_temp.data[j][i] != 0.0){
                    Subtract_row(Mat_temp.data[i], Mat_temp.data[j], n);
                    Subtract_row(Res->data[i], Res->data[j], n);
                    break;
                }
            }
        }
        for (uint_fast8_t j = 0; j != i; j++){ // Iterate along row length
            if ((Mat_temp.data[i][j] != 0) && (Mat_temp.data[j][j] == 1)){
                // subtract from both sides row j multiplied by left hand matrix [i][j]
                double temp[n];
                double temp_con = Mat_temp.data[i][j];
                memcpy(temp, Res->data[j], n*sizeof(double));
                for (uint_fast8_t k = 0; k != n; k++){Res->data[i][k] -= (temp[k]*temp_con);}
                memcpy(temp, Mat_temp.data[j], n*sizeof(double));
                for (uint_fast8_t k = 0; k != n; k++){Mat_temp.data[i][k] -= (temp[k]*temp_con);}
            }
        }
        if ((Mat_temp.data[i][i] != 1)&&(Mat_temp.data[i][i] != 0)){
            Multiply_row(Res->data[i], (1.0/Mat_temp.data[i][i]), n);
            Multiply_row(Mat_temp.data[i], (1.0/Mat_temp.data[i][i]), n);
        }
    }
    // Now work backwards to convert matrix to reduced row echelon form 
    for (int_fast16_t i = n - 1; i >= 0; i--){
        for (uint_fast8_t j = n - 1; j > i; j--){
            if ((Mat_temp.data[i][j] != 0) && (Mat_temp.data[j][j] == 1)){
                // subtract from both sides row j multiplied by left hand matrix [i][j]
                double temp[n];
                double temp_con = Mat_temp.data[i][j];
                memcpy(temp, Res->data[j], n*sizeof(double));
                for (uint_fast8_t k = 0; k != n; k++){Res->data[i][k] -= (temp[k]*temp_con);}
                memcpy(temp, Mat_temp.data[j], n*sizeof(double));
                for (uint_fast8_t k = 0; k != n; k++){Mat_temp.data[i][k] -= (temp[k]*temp_con);}
            }
        }
    }
    
    // Double check result, Mat_temp should now be an identity matrix
    for (uint_fast8_t i = 0; i != n; i++){
        for (uint_fast8_t j = 0; j != n; j++){
            double check_val = (i == j)?(fabs(Mat_temp.data[i][j] - 1.0)):fabs(Mat_temp.data[i][j]);
            if (check_val > Mat_inv_threshold){
                return false;
            }
        }
    }
    
    return true;
}
