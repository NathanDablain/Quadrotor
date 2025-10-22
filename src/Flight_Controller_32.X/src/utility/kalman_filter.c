#include "kalman_filter.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

Kalman_Filter* Filter_Constructor(uint8_t states, uint8_t inputs, uint8_t measurements){
    Kalman_Filter *filter = (Kalman_Filter *)malloc(sizeof(Kalman_Filter));
    if (filter == NULL) return NULL;
    
    uint8_t count = 0;
    filter->F = Mat_Constructor(states, states);
    if (filter->F == NULL){
        Filter_Destructor(filter, count++);
        return NULL;
    }
    
    filter->B = Mat_Constructor(states, inputs);
    if (filter->B == NULL){
        Filter_Destructor(filter, count++);
        return NULL;
    }
    
    filter->H = Mat_Constructor(measurements, states);
    if (filter->H == NULL){
        Filter_Destructor(filter, count++);
        return NULL;
    }
    
    filter->Q = Mat_Constructor(states, states);
    if (filter->Q == NULL){
        Filter_Destructor(filter, count++);
        return NULL;
    }
    
    filter->R = Mat_Constructor(measurements, measurements);
    if (filter->R == NULL){
        Filter_Destructor(filter, count++);
        return NULL;
    }
    
    filter->P = Mat_Constructor(states, states);
    if (filter->P == NULL){
        Filter_Destructor(filter, count++);
        return NULL;
    }

    filter->K = Mat_Constructor(states, measurements);
    if (filter->K == NULL){
        Filter_Destructor(filter, count++);
        return NULL;
    }

    filter->xhat = Mat_Constructor(states, 1);
    if (filter->xhat == NULL){
        Filter_Destructor(filter, count);
        return NULL;
    }
    
    return filter;
}

void Filter_Destructor(Kalman_Filter *filter, uint8_t stage){
    Mat_Destructor(filter->F);
    if (stage > 0) Mat_Destructor(filter->B);
    if (stage > 1) Mat_Destructor(filter->H);
    if (stage > 2) Mat_Destructor(filter->Q);
    if (stage > 3) Mat_Destructor(filter->R);
    if (stage > 4) Mat_Destructor(filter->P);
    if (stage > 5) Mat_Destructor(filter->K); 
    if (stage > 6) Mat_Destructor(filter->xhat);
    free(filter);
}

bool Predict(Kalman_Filter *filter, Matrix *input){
    // xhat_new = F*xhat_old + B*u
    Matrix *xhat_new = Mat_Add(Mat_Mul(filter->F, filter->xhat, 0), Mat_Mul(filter->B, input, 0), 3);
    // We could free memory used by filter->xhat in call to Mat_Add, but hold off to make sure entire
    // memory allocation was successful before throwing out old states
    if (xhat_new != NULL){
        Mat_Destructor(filter->xhat);
        filter->xhat = xhat_new;
    }
    else{
        return false;
    }

    Matrix *P_new = Mat_Add(Mat_Mul(Mat_Mul(filter->F, filter->P, 0), Mat_Tran(filter->F), 3), filter->Q, 1);

    if (P_new != NULL){
        Mat_Destructor(filter->P);
        filter->P = P_new;
    }
    else{
        return false;
    }

    return true;
}

bool Predict_EKF(Kalman_Filter *filter){
    // In the extended Kalman Filter predict step it is assumed that the system nonlinear dynamics have been propogated externally

    Matrix *P_new = Mat_Add(Mat_Mul(Mat_Mul(filter->F, filter->P, 0), Mat_Tran(filter->F), 3), filter->Q, 1);
    if (P_new != NULL){
        Mat_Destructor(filter->P);
        filter->P = P_new;
    }
    else{
        return false;
    }

    return true;
}

bool Update(Kalman_Filter *filter, Matrix *measurement){
    Matrix *ybar = Mat_Sub(measurement, Mat_Mul(filter->H, filter->xhat, 0), 2);
    if (ybar == NULL) return false;

    Matrix *H_tran = Mat_Tran(filter->H);
    if (H_tran == NULL){
        Mat_Destructor(ybar);
        return false;
    }

    Matrix *S = Mat_Add(Mat_Mul(Mat_Mul(filter->H, filter->P, 0), H_tran, 1), filter->R, 1);
    if (S == NULL){
        Mat_Destructor(ybar);
        return false;
    }

    Matrix *S_inv = Mat_Inv(S);
    Mat_Destructor(S);
    if (S_inv == NULL){
        Mat_Destructor(ybar);
        return false;
    }
    for (uint8_t i = 0; i < 3; i++){
        for (uint8_t j = 0; j < 3; j++){
            printf("%f  ", S_inv->data[i][j]);
        }
        printf("\n");
    }
    Matrix *K_new = Mat_Mul(Mat_Mul(filter->P, H_tran, 2), S_inv, 3);
    if (K_new != NULL){
        Mat_Destructor(filter->K);
        filter->K = K_new;
    }
    else{
        Mat_Destructor(ybar);
        return false;
    }

    Matrix *x_new = Mat_Add(filter->xhat, Mat_Mul(filter->K, ybar, 2), 2);

    if (x_new != NULL){
        Mat_Destructor(filter->xhat);
        filter->xhat = x_new;
    }
    else{
        return false;
    }

    Matrix *P_new = Mat_Mul(Mat_Sub(Mat_Id(filter->xhat->rows), Mat_Mul(filter->K, filter->H, 0), 3) , filter->P, 1);

    if (P_new != NULL){
        Mat_Destructor(filter->P);
        filter->P = P_new;
        return true;
    }
    else{
        return false;
    }
    
}

bool Update_EKF(Kalman_Filter *filter, Matrix *measurement, Matrix *predicted_measurement){
    Matrix *ybar = Mat_Sub(measurement, predicted_measurement, 2);
    if (ybar == NULL) return false;

    Matrix *H_tran = Mat_Tran(filter->H);
    if (H_tran == NULL){
        Mat_Destructor(ybar);
        return false;
    }

    Matrix *S = Mat_Add(Mat_Mul(Mat_Mul(filter->H, filter->P, 0), H_tran, 1), filter->R, 1);
    if (S == NULL){
        Mat_Destructor(ybar);
        return false;
    }

    Matrix *S_inv = Mat_Inv(S);
    Mat_Destructor(S);
    if (S_inv == NULL){
        Mat_Destructor(ybar);
        return false;
    }

    Matrix *K = Mat_Mul(Mat_Mul(filter->P, H_tran, 2), S_inv, 3);

    if (K == NULL){
        Mat_Destructor(ybar);
        return false;
    }

    Matrix *x_new = Mat_Add(filter->xhat, Mat_Mul(K, ybar, 2), 2);

    if (x_new != NULL){
        Mat_Destructor(filter->xhat);
        filter->xhat = x_new;
    }
    else{
        return false;
    }

    Matrix *P_new = Mat_Mul(Mat_Sub(Mat_Id(filter->xhat->rows), Mat_Mul(K, filter->H, 1), 3) , filter->P, 1);
    if (P_new != NULL){
        Mat_Destructor(filter->P);
        filter->P = P_new;
        return true;
    }
    else{
        return false;
    }
}
