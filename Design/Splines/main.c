#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
    float *data;
    uint8_t length;
} Vector;

typedef struct {
    float Position[3];
    float Velocity[3];
    float Time;
} Line;

void Calculate_Coefficients(Line *Points_to_interpolate, Vector *a_coeff, Vector *b_coeff, Vector *c_coeff, Vector *d_coeff, uint8_t line_index);

void Multiply_row(float *array, float con, uint8_t length);

void Subtract_row(float *left_side, float *right_side, uint8_t length);

int main (void){
    // Allocate memory for interpolation
    Line Points_to_interpolate[3] = {{.Position[0] = 100, .Velocity[0] = -5, .Time = 0.5},
                                     {.Position[0] = 80, .Velocity[0] = -5, .Time = 1.5},
                                     {.Position[0] = 30, .Velocity[0] = -8, .Time = 2.5}};
    uint8_t n = sizeof(Points_to_interpolate)/sizeof(Line);
    // for (uint8_t i = 0; i < n; i++){
    //     Points_to_interpolate[i].Position.data = calloc(3, sizeof(float*));
    //     Points_to_interpolate[i].Position.length = 3;
    //     Points_to_interpolate[i].Velocity.data = calloc(3, sizeof(float*));
    //     Points_to_interpolate[i].Velocity.length = 3;
    //     Points_to_interpolate[i].Time.data = calloc(3, sizeof(float*));
    //     Points_to_interpolate[i].Time.length = 3;
    // }
    Vector a_coeff = {.data = calloc(a_coeff.length, sizeof(*a_coeff.data)), .length = n-1};
    Vector b_coeff = {.data = calloc(b_coeff.length, sizeof(*b_coeff.data)), .length = n};
    Vector c_coeff = {.data = calloc(c_coeff.length, sizeof(*c_coeff.data)), .length = n-1};
    Vector d_coeff = {.data = calloc(d_coeff.length, sizeof(*d_coeff.data)), .length = n-1};
    // Find coefficients for splines between all points
    Calculate_Coefficients(Points_to_interpolate, &a_coeff, &b_coeff, &c_coeff, &d_coeff, 0);

    // Clean up data previously allocated
    free(a_coeff.data);
    free(b_coeff.data);
    free(c_coeff.data);
    free(d_coeff.data);
    // for (uint8_t i = 0; i < n; i++){
    //     free(Points_to_interpolate[i].Position.data);
    //     free(Points_to_interpolate[i].Velocity.data);
    //     free(Points_to_interpolate[i].Time.data);
    // }
    return 0;
}

void Calculate_Coefficients(Line *Points_to_interpolate, Vector *a_coeff, Vector *b_coeff, Vector *c_coeff, Vector *d_coeff, uint8_t line_index){
    // Given velocity at endpoints, solve using clamped method and gaussian elimination
    uint8_t n = b_coeff->length;
    float Matrix[n][n];
    memset(Matrix, 0.0, n*n*sizeof(float));
    float Solns[n];
    memset(Solns, 0.0, n*sizeof(float));
    float h[n-1];
    memset(h, 0.0, (n-1)*sizeof(float));
    float f[n-1];
    memset(f, 0.0, (n-1)*sizeof(float));
    // Solve for dx and dt for each segment we are interpolating between
    for (uint8_t i = 0; i < n-1; i++){
        h[i] = Points_to_interpolate[i+1].Time -  Points_to_interpolate[i].Time;
        f[i] = Points_to_interpolate[i+1].Position[line_index] - Points_to_interpolate[i].Position[line_index];
    }
    // Set up system of linear equations for gaussian elimination
    for (uint8_t i = 0; i < n; i++){
        if (i == 0){
            Matrix[i][0] = (2.0/3.0)*h[0];
            Matrix[i][1] = (1.0/3.0)*h[0];
            Solns[i] = (f[0]/h[0]) - Points_to_interpolate[i].Velocity[line_index];
        }
        else if (i == n-1){
            Matrix[i][n-2] = (1.0/3.0)*h[i-1];
            Matrix[i][n-1] = (2.0/3.0)*h[i-1];
            Solns[i] = Points_to_interpolate[i].Velocity[line_index] - (f[i-1]/h[i-1]);
        }
        else{
            Matrix[i][i-1] = (1.0/3.0)*h[i-1]; // b_i-1
            Matrix[i][i] = (2.0/3.0)*(h[i-1] + h[i]); // b_i
            Matrix[i][i+1] = (1.0/3.0)*h[i]; // b_i+1
            Solns[i] = (f[i]/h[i]) - (f[i-1]/h[i-1]);
        }
    }

    // Use gauss-jordan elimination to solve system of linear equations
    // First work along rows until you get row echelon form, then work backwards to get reduced row echelon form
    // Search along row until you get to the diagonal entry, if any values before this entry are not zero, subtract their value*
    for (uint8_t i = 0; i < n; i++){ // Iterate down rows
        for (uint8_t j = 0; j < i; j++){ // Iterate along row length
            if ((Matrix[i][j] != 0) && (Matrix[j][j] == 1)){
                Solns[i] -= (Solns[j]*Matrix[i][j]);
                float temp[n];
                memcpy(temp, Matrix[j], n*sizeof(float));
                Multiply_row(temp, Matrix[i][j], n);
                Subtract_row(Matrix[i], temp, n);
            }
        }
        if ((Matrix[i][i] != 1)&&(Matrix[i][i] != 0)){
            Solns[i] /= Matrix[i][i];
            Multiply_row(Matrix[i], (1.0/Matrix[i][i]), n);
        }
    }
    // Now work backwards to convert matrix to reduced row echelon form 
    for (uint8_t i = n-1; i>=0; i--){
        for (uint8_t j = n-1; j>i; j--){
            if ((Matrix[i][j] != 0) &&(Matrix[j][j] == 1)){
                Solns[i] -= (Solns[j]*Matrix[i][j]);
                float temp[n];
                memcpy(temp, Matrix[j], n*sizeof(float));
                Multiply_row(temp, Matrix[i][j], n);
                Subtract_row(Matrix[i], temp, n);
            }
        }
    }
}

void Multiply_row(float *array, float con, uint8_t length){
    for (uint8_t i = 0; i<length; i++){
        array[i] *= con;
    }
}

void Subtract_row(float *left_side, float *right_side, uint8_t length){
    for (uint8_t i = 0; i<length; i++){
        left_side[i] -= right_side[i];
    }
}