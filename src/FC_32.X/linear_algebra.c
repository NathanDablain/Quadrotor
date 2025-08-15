#include "linear_algebra.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

Matrix* Mat_Constructor(uint8_t rows, uint8_t columns){
    Matrix* Mat_new = (Matrix *)malloc(sizeof(Matrix));
    if (Mat_new == NULL) return NULL;

    Mat_new->data = (float **)malloc(rows*sizeof(float*));
    Mat_new->rows = rows;
    if (Mat_new->data == NULL){
        free(Mat_new);
        return NULL;
    }
    for (uint_fast8_t i = 0; i != rows; i++){
        Mat_new->data[i] = (float *)calloc(columns, sizeof(float));
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
    for (uint_fast8_t i = 0; i != Mat->columns; i++){
        free(Mat->data[i]);
    }
    free(Mat->data);
    free(Mat);
}

Matrix* Mat_Add(Matrix* Mat1, Matrix* Mat2, uint8_t Destroy_flag){
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

Matrix* Mat_Sub(Matrix* Mat1, Matrix* Mat2, uint8_t Destroy_flag){
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

Matrix* Mat_Mul(Matrix* Mat1, Matrix* Mat2, uint8_t Destroy_flag){
    // Ensure matrix 1 has the same number of columns as matrix 2 has rows
    if (Mat1->columns != Mat2->rows) return NULL;
    // Build resultant matrix
    Matrix* Mat_result = Mat_Constructor(Mat1->rows, Mat2->columns);
    if (Mat_result == NULL) return NULL;

    for (uint_fast8_t i = 0; i != Mat1->rows; i++){
        for (uint_fast8_t j = 0; j != Mat2->columns; j++){
            for (uint_fast8_t k = 0; k != Mat1->columns; k++){
                float temp1 = Mat1->data[i][k];
                float temp2 = Mat2->data[k][j];
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

Matrix* Mat_Tran(Matrix* Mat){
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

Matrix* Mat_Inv(Matrix* Mat){
    // Ensure matrix has the same number of columns as rows
    if (Mat->columns != Mat->rows) return NULL;
    // Mat temp will be used to hold the values of the input Mat for this functions manipulation
    Matrix Mat_temp = *Mat;
    // Mat result will begin as the identity matrix and become the inverse of Mat through elementary row operations
    Matrix* Mat_result = Mat_Id(Mat->rows);
    if (Mat_result == NULL) return NULL;

    uint8_t n = Mat->rows;
    const float Mat_inv_threshold = 1e-3;

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
                float temp[n];
                float temp_con = Mat_temp.data[i][j];
                memcpy(temp, Mat_result->data[j], n*sizeof(float));
                for (uint_fast8_t k = 0; k != n; k++){Mat_result->data[i][k] -= (temp[k]*temp_con);}
                memcpy(temp, Mat_temp.data[j], n*sizeof(float));
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
                float temp[n];
                float temp_con = Mat_temp.data[i][j];
                memcpy(temp, Mat_result->data[j], n*sizeof(float));
                for (uint_fast8_t k = 0; k != n; k++){Mat_result->data[i][k] -= (temp[k]*temp_con);}
                memcpy(temp, Mat_temp.data[j], n*sizeof(float));
                for (uint_fast8_t k = 0; k != n; k++){Mat_temp.data[i][k] -= (temp[k]*temp_con);}
            }
        }
    }
    
    // Double check result, Mat_temp should now be an identity matrix
    for (uint_fast8_t i = 0; i != n; i++){
        for (uint_fast8_t j = 0; j != n; j++){
            float check_val = (i == j)?(fabs(Mat_temp.data[i][j] - 1.0)):fabs(Mat_temp.data[i][j]);
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

void Mult_and_Sub_Row(float *left_side, float *right_side, float con, uint8_t length){
    for (uint_fast8_t i = 0; i != length; i++){
        left_side[i] -= right_side[i]*con; 
    }
}

void Multiply_row(float *row, float con, uint8_t length){
    for (uint_fast8_t i = 0; i != length; i++){
        row[i] *= con;
    }
}

void Subtract_row(float *left_side, float *right_side, uint8_t length){
    for (uint_fast8_t i = 0; i != length; i++){
        left_side[i] -= right_side[i];
    }
}
