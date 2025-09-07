#include "kalman_filter.h"
#include "stdlib.h"

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
    
    filter->R = Mat_Constructor(inputs, inputs);
    if (filter->R == NULL){
        Filter_Destructor(filter, count++);
        return NULL;
    }
    
    filter->P = Mat_Constructor(states, states);
    if (filter->P == NULL){
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
    if (stage > 5) Mat_Destructor(filter->xhat);
    free(filter);
}

bool Predict(Kalman_Filter *filter, Matrix *input){
    // xhat_new = F*xhat_old + B*u, toss F*xhat_old and B*u
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
    // P_new = F*P*F' + Q, toss F*P, F', and F*P*F'
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
    // ybar = y - H*xhat, toss H*xhat 
    Matrix *ybar = Mat_Sub(measurement, Mat_Mul(filter->H, filter->xhat, 0), 2);
    // S = H*P*H' + R, toss H*P, H', and H*P*H'
    Matrix *S = Mat_Add(Mat_Mul(Mat_Mul(filter->H, filter->P, 0), Mat_Tran(filter->H), 3), filter->R, 1);
    // K = P*H'*inv(S), toss H', P*H', and inv(S)
    Matrix *K = Mat_Mul(Mat_Mul(filter->P, Mat_Tran(filter->H), 2), Mat_Inv(S), 3);
    // x_new = xhat + K*ybar, toss K*ybar and ybar
    Matrix *x_new = Mat_Add(filter->xhat, Mat_Mul(K, ybar, 2), 2);
    if (x_new != NULL){
        Mat_Destructor(filter->xhat);
        filter->xhat = x_new;
    }
    else{
        return false;
    }
    // P_new = (I - K*H)*P, toss I, K, K*H, and I - K*H
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