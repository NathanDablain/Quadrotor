#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "navigation.h"
#include "linear_algebra.h"
#include "time.h"
#include "imu.h"
#include "magnetometer.h"
#include "global_variables.h"

// In units of Hz
static const double Gyro_ODR = 1660.0;
static const double Gyro_ODR_2 = 40.743097574926725;
static const double d_t = 1.0/Gyro_ODR;
// Date sheet gives value of 5mdps/sqrt(ODR), convert to rad/s
static const double Gyro_RMS_Noise = 0.005*Gyro_ODR_2*D2R;
// In units of Hz
static const double Accel_ODR_2 = 40.743097574926725;
// Data sheet gives value of 90ug/sqrt(ODR), convert to g
static const double Accel_RMS_Noise = 0.00009*Accel_ODR_2;
// Assume drift rate of 20 deg/hr, convert to rad/s
static const double Gyro_Bias_Instability = (20.0/3600.0)*D2R;
// Constant value if LPF = ODR/2, normalize by dividing by average field strength
static const double Mag_RMS_Noise = 4.5/500.0;
// Holds ground filter estimated states
static double Ground_Filter_xhat[9];
static double Air_Filter_xhat[6];
static Ground_Filter_Covariance P_GF;
static Air_Filter_Covariance P_AF;
static const Ground_Filter_Covariance G_Process_Cov = {
    .P1_1 = Gyro_RMS_Noise,
    .P2_2 = Gyro_RMS_Noise,
    .P3_3 = Gyro_RMS_Noise,
    .P4_4 = Gyro_Bias_Instability,
    .P5_5 = Gyro_Bias_Instability,
    .P6_6 = Gyro_Bias_Instability,
    .P7_7 = Gyro_RMS_Noise*d_t,
    .P7_8 = Gyro_RMS_Noise*d_t,
    .P7_9 = Gyro_RMS_Noise*d_t,
    .P8_8 = Gyro_RMS_Noise*d_t,
    .P8_9 = Gyro_RMS_Noise*d_t,
    .P9_9 = Gyro_RMS_Noise*d_t
};
static const Air_Filter_Covariance A_Process_Cov = {
    .P4_4 = Gyro_RMS_Noise*d_t,
    .P4_5 = Gyro_RMS_Noise*d_t,
    .P4_6 = Gyro_RMS_Noise*d_t,
    .P5_5 = Gyro_RMS_Noise*d_t,
    .P5_6 = Gyro_RMS_Noise*d_t,
    .P6_6 = Gyro_RMS_Noise*d_t
};

void Run_Ground_Filter(bool Initialize){
    const Time G_Filter_Predict_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/1660};
    static Time G_Filter_Predict_Last;
    const Time G_Filter_Update_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/100};
    static Time G_Filter_Update_Last;
    
    if (Initialize){
        Initialize_Ground_Filter();
        G_Filter_Predict_Last = Current_Time();
        G_Filter_Update_Last = Current_Time();
    }
    else{
        if (Compare_And_Update(Current_Time(), G_Filter_Predict_Rate, &G_Filter_Predict_Last)){
            Ground_Filter_Predict();
        }
        if (Compare_And_Update(Current_Time(), G_Filter_Update_Rate, &G_Filter_Update_Last)){
            Ground_Filter_Update();
        }
    }
}

void Run_Air_Filter(bool Initialize){
    const Time A_Filter_Predict_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/1660};
    static Time A_Filter_Predict_Last;
    const Time A_Filter_Update_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/100};
    static Time A_Filter_Update_Last;
    
    if (Initialize){
        Initialize_Air_Filter();
        A_Filter_Predict_Last = Current_Time();
        A_Filter_Update_Last = Current_Time();
    }
    else{
        if (Compare_And_Update(Current_Time(), A_Filter_Predict_Rate, &A_Filter_Predict_Last)){
            Air_Filter_Predict();
        }
        if (Compare_And_Update(Current_Time(), A_Filter_Update_Rate, &A_Filter_Update_Last)){
            Air_Filter_Update();
        }
    }
}

void Initialize_Ground_Filter(){
    memset(Ground_Filter_xhat, 0, sizeof(Ground_Filter_xhat));
    memset(&P_GF, 0, sizeof(P_GF));

    // Seed bias uncertainties so that they will converge faster
    P_GF.P4_4 = 1.0;
    P_GF.P5_5 = 1.0;
    P_GF.P6_6 = 1.0;
    P_GF.P7_7 = 1.0;
    P_GF.P8_8 = 1.0;
    P_GF.P9_9 = 1.0;
}

// Before take off, assume that normal forces acting on the drone enable measuring
// of the inverse gravity vector
// An initial Kalman filter will be run under these assumptions in order to estimate
// initial attitude conditions and gyro bias instability

// The ground Kalman filter will have 9 states, 3 inputs, and 3 measurements
// States are : w_x, w_y, w_z, w_x_bias, w_y_bias, w_z_bias, phi, theta, psi
// Inputs are : p, q, r
// Measurements are: phi, theta, psi
void Ground_Filter_Predict(){
    const double d_t2 = pow(d_t, 2);
    const double pi_2 = M_PI/2.0;
    // If we are too close to gimbal lock, dont do phi and psi prediction
    const double theta_min = 0.0873;
    double x_new[9] = {0};
    Ground_Filter_Covariance P_new;
    double s_phi = sin(Ground_Filter_xhat[6]);
    double c_phi = cos(Ground_Filter_xhat[6]);
    // Propogate state estimates
    x_new[0] = IMU_Angular_Rate(0)*D2R - Ground_Filter_xhat[3];
    x_new[1] = IMU_Angular_Rate(1)*D2R - Ground_Filter_xhat[4];
    x_new[2] = IMU_Angular_Rate(2)*D2R - Ground_Filter_xhat[5];
    x_new[3] = Ground_Filter_xhat[3];
    x_new[4] = Ground_Filter_xhat[4];
    x_new[5] = Ground_Filter_xhat[5];
    x_new[7] = Ground_Filter_xhat[7] + d_t*(Ground_Filter_xhat[1]*c_phi - Ground_Filter_xhat[2]*s_phi);
    if (abs(pi_2 - Ground_Filter_xhat[7]) >  theta_min){
        double t_theta = tan(Ground_Filter_xhat[7]);
        double c_theta = cos(Ground_Filter_xhat[7]);
        x_new[6] = Ground_Filter_xhat[6] + d_t*(Ground_Filter_xhat[0] + Ground_Filter_xhat[1]*s_phi*t_theta + Ground_Filter_xhat[2]*c_phi*t_theta); 
        x_new[8] = Ground_Filter_xhat[8] + d_t*((Ground_Filter_xhat[1]*s_phi/c_theta) + (Ground_Filter_xhat[2]*c_phi/c_theta)); 
        
        // Propagate covariance estimates
        P_new.P1_1 = P_GF.P4_4 + G_Process_Cov.P1_1;
        P_new.P1_2 = P_GF.P4_5;
        P_new.P1_3 = P_GF.P4_6;
        P_new.P1_4 = -P_GF.P4_4;
        P_new.P1_5 = -P_GF.P4_5;
        P_new.P1_6 = -P_GF.P4_6;
        P_new.P1_7 = -P_GF.P4_7 - d_t*P_GF.P1_4 - d_t*P_GF.P3_4*c_phi*t_theta - d_t*P_GF.P2_4*s_phi*t_theta;
        P_new.P1_8 = d_t*P_GF.P3_4*s_phi - d_t*P_GF.P2_4*c_phi - P_GF.P4_8;
        P_new.P1_9 = -P_GF.P4_9 - (d_t*P_GF.P3_4*c_phi/c_theta) - (d_t*P_GF.P2_4*s_phi/c_theta);

        P_new.P2_2 = P_GF.P5_5 + G_Process_Cov.P2_2;
        P_new.P2_3 = P_GF.P5_6;
        P_new.P2_4 = -P_GF.P4_5;
        P_new.P2_5 = -P_GF.P5_5;
        P_new.P2_6 = -P_GF.P5_6;
        P_new.P2_7 = -P_GF.P5_7 - d_t*P_GF.P1_5 - d_t*P_GF.P3_5*c_phi*t_theta - d_t*P_GF.P2_5*s_phi*t_theta;
        P_new.P2_8 = d_t*P_GF.P3_5*s_phi - d_t*P_GF.P2_5*c_phi - P_GF.P5_8;
        P_new.P2_9 = -P_GF.P5_9 - (d_t*P_GF.P3_5*c_phi/c_theta) - (d_t*P_GF.P2_5*s_phi/c_theta);

        P_new.P3_3 = P_GF.P6_6 + G_Process_Cov.P3_3;
        P_new.P3_4 = -P_GF.P4_6;
        P_new.P3_5 = -P_GF.P5_6;
        P_new.P3_6 = -P_GF.P6_6;
        P_new.P3_7 = -P_GF.P6_7 - d_t*P_GF.P1_6 - d_t*P_GF.P3_6*c_phi*t_theta - d_t*P_GF.P2_6*s_phi*t_theta;
        P_new.P3_8 = d_t*P_GF.P3_6*s_phi - d_t*P_GF.P2_6*c_phi - P_GF.P6_8;
        P_new.P3_9 = -P_GF.P6_9 - (d_t*P_GF.P3_6*c_phi/c_theta) - (d_t*P_GF.P2_6*s_phi/c_theta);

        P_new.P4_4 = P_GF.P4_4 + G_Process_Cov.P4_4;
        P_new.P4_5 = P_GF.P4_5;
        P_new.P4_6 = P_GF.P4_6;
        P_new.P4_7 = P_GF.P4_7 + d_t*P_GF.P1_4 + d_t*P_GF.P3_4*c_phi*t_theta + d_t*P_GF.P2_4*s_phi*t_theta;
        P_new.P4_8 = d_t*P_GF.P2_4*c_phi - d_t*P_GF.P3_4*s_phi + P_GF.P4_8;
        P_new.P4_9 = P_GF.P4_9 + (d_t*P_GF.P3_4*c_phi/c_theta) + (d_t*P_GF.P2_4*s_phi/c_theta);

        P_new.P5_5 = P_GF.P5_5 + G_Process_Cov.P5_5;
        P_new.P5_6 = P_GF.P5_6;
        P_new.P5_7 = P_GF.P5_7 + d_t*P_GF.P1_5 + d_t*P_GF.P3_5*c_phi*t_theta + d_t*P_GF.P2_5*s_phi*t_theta;
        P_new.P5_8 = d_t*P_GF.P2_5*c_phi - d_t*P_GF.P3_5*s_phi + P_GF.P5_8;
        P_new.P5_9 = P_GF.P5_9 + (d_t*P_GF.P3_5*c_phi/c_theta) + (d_t*P_GF.P2_5*s_phi/c_theta);

        P_new.P6_6 = P_GF.P6_6 + G_Process_Cov.P6_6;
        P_new.P6_7 = P_GF.P6_7 + d_t*P_GF.P1_6 + d_t*P_GF.P3_6*c_phi*t_theta + d_t*P_GF.P2_6*s_phi*t_theta;
        P_new.P6_8 = d_t*P_GF.P2_6*c_phi - d_t*P_GF.P3_6*s_phi + P_GF.P6_8;
        P_new.P6_9 = P_GF.P6_9 + (d_t*P_GF.P3_6*c_phi/c_theta) + (d_t*P_GF.P2_6*s_phi/c_theta);

        P_new.P7_7 = P_GF.P7_7 + 2*d_t + 2*d_t*P_GF.P1_7 + d_t2*P_GF.P1_1 + d_t2*P_GF.P3_3*pow(c_phi, 2)*pow(t_theta, 2) +
                     d_t2*P_GF.P2_2*pow(s_phi, 2)*pow(t_theta, 2) + 2.0*d_t2*P_GF.P1_3*c_phi*t_theta +
                     2.0*d_t2*P_GF.P1_2*s_phi*t_theta + 2.0*d_t2*P_GF.P2_3*c_phi*s_phi*pow(t_theta, 2) + 
                     2.0*d_t*P_GF.P3_7*c_phi*t_theta + 2.0*d_t*P_GF.P2_7*s_phi*t_theta + G_Process_Cov.P7_7;
        P_new.P7_8 = P_GF.P7_8 + d_t*P_GF.P1_8 + d_t*c_phi*(P_GF.P2_7 + P_GF.P1_2*d_t + P_GF.P2_3*d_t*c_phi*t_theta +
                     P_GF.P2_2*d_t*s_phi*t_theta) - d_t*s_phi*(P_GF.P3_7 + P_GF.P1_3*d_t + P_GF.P3_3*d_t*c_phi*t_theta +
                     P_GF.P2_3*d_t*s_phi*t_theta) + P_GF.P3_8*d_t*c_phi*t_theta + P_GF.P2_8*d_t*s_phi*t_theta + G_Process_Cov.P7_8;
        P_new.P7_9 = P_GF.P7_9 + d_t*P_GF.P1_9 + ((d_t*c_phi*(P_GF.P3_7 + P_GF.P1_3*d_t + P_GF.P3_3*d_t*c_phi*t_theta + 
                     P_GF.P2_3*d_t*s_phi*t_theta))/c_theta) + ((d_t*s_phi*(P_GF.P2_7 + d_t*P_GF.P1_2 + d_t*P_GF.P2_3*c_phi*t_theta +
                     d_t*P_GF.P2_2*s_phi*t_theta))/c_theta) + P_GF.P3_9*d_t*c_phi*t_theta + P_GF.P2_9*d_t*s_phi*t_theta + G_Process_Cov.P7_9;

        P_new.P8_8 = P_GF.P8_8 + 2.0*d_t + P_GF.P2_2*d_t2*pow(c_phi, 2) + P_GF.P3_3*d_t2*pow(s_phi, 2) + 2.0*P_GF.P2_8*d_t*c_phi -
                     2.0*P_GF.P3_8*d_t*s_phi - 2.0*P_GF.P2_3*d_t2*c_phi*s_phi + G_Process_Cov.P8_8;
        P_new.P8_9 = P_GF.P8_9 + P_GF.P2_9*d_t*c_phi - P_GF.P3_9*d_t*s_phi + (d_t*c_phi*(P_GF.P3_8 + P_GF.P2_3*d_t*c_phi - 
                     P_GF.P3_3*d_t*s_phi))/c_theta + (d_t*s_phi*(P_GF.P2_8 + P_GF.P2_2*d_t*c_phi - P_GF.P2_3*d_t*s_phi))/c_theta + G_Process_Cov.P8_9;

        P_new.P9_9 = P_GF.P9_9 + 5.0*d_t + P_GF.P3_3*d_t2*pow(c_phi, 2)/pow(c_theta, 2) + P_GF.P2_2*d_t2*pow(s_phi, 2)/pow(c_theta, 2) +
                     2.0*P_GF.P2_3*d_t2*c_phi*s_phi/pow(c_theta, 2) + 2.0*P_GF.P3_9*d_t*c_phi/c_theta + 2.0*P_GF.P2_9*d_t*s_phi/c_theta + G_Process_Cov.P9_9;


        P_GF = P_new;
    }
    memcpy(Ground_Filter_xhat, x_new, sizeof(x_new));

}

void Ground_Filter_Update(){
    Matrix_3 S = {0};
    Matrix_3 S_inv = {0};
    double R[3] = {Accel_RMS_Noise, Accel_RMS_Noise, Mag_RMS_Noise};
    // Matrix_3 S;
    double measurements[3];
    double phi = Ground_Filter_xhat[6];
    double theta = Ground_Filter_xhat[7];
    double s_phi = sin(phi);
    double c_phi = cos(phi);

    // Perform roll-pitch-yaw measurements
    measurements[0] = atan2((-IMU_Acceleration(1)), (-IMU_Acceleration(2)));
    if (isnan(measurements[0])){
        measurements[0] = phi;
    }
    measurements[1] = atan2((IMU_Acceleration(0)), sqrt(pow(IMU_Acceleration(1),2) + pow(IMU_Acceleration(2),2)));
    if (isnan(measurements[1])){
        measurements[1] = theta;
    }

    double mag_x_NED = cos(theta)*Magnetometer_Field(0) + s_phi*sin(theta)*Magnetometer_Field(1) + c_phi*sin(theta)*Magnetometer_Field(2);
    double mag_y_NED = c_phi*Magnetometer_Field(1) - s_phi*Magnetometer_Field(2);
    
    measurements[2] = -atan2(mag_y_NED, mag_x_NED);
    if (isnan(measurements[2])){
        measurements[2] = Ground_Filter_xhat[8];
    }
    
    double ybar[3] = {
        measurements[0] - Ground_Filter_xhat[6],
        measurements[1] - Ground_Filter_xhat[7],
        measurements[2] - Ground_Filter_xhat[8]
    };

    // Calculate innovation
    S.data[0][0] = P_GF.P7_7 + R[0];
    S.data[0][1] = P_GF.P7_8;
    S.data[0][2] = P_GF.P7_9;
    S.data[1][0] = P_GF.P7_8;
    S.data[1][1] = P_GF.P8_8 + R[1];
    S.data[1][2] = P_GF.P8_9;
    S.data[2][0] = P_GF.P7_9;
    S.data[2][1] = P_GF.P8_9;
    S.data[2][2] = P_GF.P9_9 + R[2];

    bool S_inv_successful = Mat3_Inv(&S, &S_inv);
    if (!S_inv_successful) return;

    // Compute Kalman Gain
    double K[9][3] = {{P_GF.P1_7*S_inv.data[0][0] + P_GF.P1_8*S_inv.data[0][1] + P_GF.P1_9*S_inv.data[0][2],
                       P_GF.P1_7*S_inv.data[0][1] + P_GF.P1_8*S_inv.data[1][1] + P_GF.P1_9*S_inv.data[1][2],
                       P_GF.P1_7*S_inv.data[0][2] + P_GF.P1_8*S_inv.data[1][2] + P_GF.P1_9*S_inv.data[2][2]},
                      {P_GF.P2_7*S_inv.data[0][0] + P_GF.P2_8*S_inv.data[0][1] + P_GF.P2_9*S_inv.data[0][2],
                       P_GF.P2_7*S_inv.data[0][1] + P_GF.P2_8*S_inv.data[1][1] + P_GF.P2_9*S_inv.data[1][2],
                       P_GF.P2_7*S_inv.data[0][2] + P_GF.P2_8*S_inv.data[1][2] + P_GF.P2_9*S_inv.data[2][2]},
                      {P_GF.P3_7*S_inv.data[0][0] + P_GF.P3_8*S_inv.data[0][1] + P_GF.P3_9*S_inv.data[0][2],
                       P_GF.P3_7*S_inv.data[0][1] + P_GF.P3_8*S_inv.data[1][1] + P_GF.P3_9*S_inv.data[1][2],
                       P_GF.P3_7*S_inv.data[0][2] + P_GF.P3_8*S_inv.data[1][2] + P_GF.P3_9*S_inv.data[2][2]},
                      {P_GF.P4_7*S_inv.data[0][0] + P_GF.P4_8*S_inv.data[0][1] + P_GF.P4_9*S_inv.data[0][2],
                       P_GF.P4_7*S_inv.data[0][1] + P_GF.P4_8*S_inv.data[1][1] + P_GF.P4_9*S_inv.data[1][2],
                       P_GF.P4_7*S_inv.data[0][2] + P_GF.P4_8*S_inv.data[1][2] + P_GF.P4_9*S_inv.data[2][2]},
                      {P_GF.P5_7*S_inv.data[0][0] + P_GF.P5_8*S_inv.data[0][1] + P_GF.P5_9*S_inv.data[0][2],
                       P_GF.P5_7*S_inv.data[0][1] + P_GF.P5_8*S_inv.data[1][1] + P_GF.P5_9*S_inv.data[1][2],
                       P_GF.P5_7*S_inv.data[0][2] + P_GF.P5_8*S_inv.data[1][2] + P_GF.P5_9*S_inv.data[2][2]},
                      {P_GF.P6_7*S_inv.data[0][0] + P_GF.P6_8*S_inv.data[0][1] + P_GF.P6_9*S_inv.data[0][2],
                       P_GF.P6_7*S_inv.data[0][1] + P_GF.P6_8*S_inv.data[1][1] + P_GF.P6_9*S_inv.data[1][2],
                       P_GF.P6_7*S_inv.data[0][2] + P_GF.P6_8*S_inv.data[1][2] + P_GF.P6_9*S_inv.data[2][2]},
                      {P_GF.P7_7*S_inv.data[0][0] + P_GF.P7_8*S_inv.data[0][1] + P_GF.P7_9*S_inv.data[0][2],
                       P_GF.P7_7*S_inv.data[0][1] + P_GF.P7_8*S_inv.data[1][1] + P_GF.P7_9*S_inv.data[1][2],
                       P_GF.P7_7*S_inv.data[0][2] + P_GF.P7_8*S_inv.data[1][2] + P_GF.P7_9*S_inv.data[2][2]},
                      {P_GF.P7_8*S_inv.data[0][0] + P_GF.P8_8*S_inv.data[0][1] + P_GF.P8_9*S_inv.data[0][2],
                       P_GF.P7_8*S_inv.data[0][1] + P_GF.P8_8*S_inv.data[1][1] + P_GF.P8_9*S_inv.data[1][2],
                       P_GF.P7_8*S_inv.data[0][2] + P_GF.P8_8*S_inv.data[1][2] + P_GF.P8_9*S_inv.data[2][2]},
                      {P_GF.P7_9*S_inv.data[0][0] + P_GF.P8_9*S_inv.data[0][1] + P_GF.P9_9*S_inv.data[0][2],
                       P_GF.P7_9*S_inv.data[0][1] + P_GF.P8_9*S_inv.data[1][1] + P_GF.P9_9*S_inv.data[1][2],
                       P_GF.P7_9*S_inv.data[0][2] + P_GF.P8_9*S_inv.data[1][2] + P_GF.P9_9*S_inv.data[2][2]}};
    
    double x_new[9];
    for (uint8_t i = 0; i < 9; i++){
        x_new[i] = Ground_Filter_xhat[i] + K[i][0]*ybar[0] + K[i][1]*ybar[1] + K[i][2]*ybar[2];
    }
    memcpy(Ground_Filter_xhat, x_new, sizeof(x_new));

    Ground_Filter_Covariance P_new;

    P_new.P1_1 = P_GF.P1_1 - K[0][0]*P_GF.P1_7 - K[0][1]*P_GF.P1_8 - K[0][2]*P_GF.P1_9;
    P_new.P1_2 = P_GF.P1_2 - K[0][0]*P_GF.P2_7 - K[0][1]*P_GF.P2_8 - K[0][2]*P_GF.P2_9;
    P_new.P1_3 = P_GF.P1_3 - K[0][0]*P_GF.P3_7 - K[0][1]*P_GF.P3_8 - K[0][2]*P_GF.P3_9;
    P_new.P1_4 = P_GF.P1_4 - K[0][0]*P_GF.P4_7 - K[0][1]*P_GF.P4_8 - K[0][2]*P_GF.P4_9;
    P_new.P1_5 = P_GF.P1_5 - K[0][0]*P_GF.P5_7 - K[0][1]*P_GF.P5_8 - K[0][2]*P_GF.P5_9;
    P_new.P1_6 = P_GF.P1_6 - K[0][0]*P_GF.P6_7 - K[0][1]*P_GF.P6_8 - K[0][2]*P_GF.P6_9;
    P_new.P1_7 = P_GF.P1_7 - K[0][0]*P_GF.P7_7 - K[0][1]*P_GF.P7_8 - K[0][2]*P_GF.P7_9;
    P_new.P1_8 = P_GF.P1_8 - K[0][0]*P_GF.P7_8 - K[0][1]*P_GF.P8_8 - K[0][2]*P_GF.P8_9;
    P_new.P1_9 = P_GF.P1_9 - K[0][0]*P_GF.P7_9 - K[0][1]*P_GF.P8_9 - K[0][2]*P_GF.P9_9;

    P_new.P2_2 = P_GF.P2_2 - K[1][0]*P_GF.P2_7 - K[1][1]*P_GF.P2_8 - K[1][2]*P_GF.P2_9;
    P_new.P2_3 = P_GF.P2_3 - K[1][0]*P_GF.P3_7 - K[1][1]*P_GF.P3_8 - K[1][2]*P_GF.P3_9;
    P_new.P2_4 = P_GF.P2_4 - K[1][0]*P_GF.P4_7 - K[1][1]*P_GF.P4_8 - K[1][2]*P_GF.P4_9;
    P_new.P2_5 = P_GF.P2_5 - K[1][0]*P_GF.P5_7 - K[1][1]*P_GF.P5_8 - K[1][2]*P_GF.P5_9;
    P_new.P2_6 = P_GF.P2_6 - K[1][0]*P_GF.P6_7 - K[1][1]*P_GF.P6_8 - K[1][2]*P_GF.P6_9;
    P_new.P2_7 = P_GF.P2_7 - K[1][0]*P_GF.P7_7 - K[1][1]*P_GF.P7_8 - K[1][2]*P_GF.P7_9;
    P_new.P2_8 = P_GF.P2_8 - K[1][0]*P_GF.P7_8 - K[1][1]*P_GF.P8_8 - K[1][2]*P_GF.P8_9;
    P_new.P2_9 = P_GF.P2_9 - K[1][0]*P_GF.P7_9 - K[1][1]*P_GF.P8_9 - K[1][2]*P_GF.P9_9;

    P_new.P3_3 = P_GF.P3_3 - K[2][0]*P_GF.P3_7 - K[2][1]*P_GF.P3_8 - K[2][2]*P_GF.P3_9;
    P_new.P3_4 = P_GF.P3_4 - K[2][0]*P_GF.P4_7 - K[2][1]*P_GF.P4_8 - K[2][2]*P_GF.P4_9;
    P_new.P3_5 = P_GF.P3_5 - K[2][0]*P_GF.P5_7 - K[2][1]*P_GF.P5_8 - K[2][2]*P_GF.P5_9;
    P_new.P3_6 = P_GF.P3_6 - K[2][0]*P_GF.P6_7 - K[2][1]*P_GF.P6_8 - K[2][2]*P_GF.P6_9;
    P_new.P3_7 = P_GF.P3_7 - K[2][0]*P_GF.P7_7 - K[2][1]*P_GF.P7_8 - K[2][2]*P_GF.P7_9;
    P_new.P3_8 = P_GF.P3_8 - K[2][0]*P_GF.P7_8 - K[2][1]*P_GF.P8_8 - K[2][2]*P_GF.P8_9;
    P_new.P3_9 = P_GF.P3_9 - K[2][0]*P_GF.P7_9 - K[2][1]*P_GF.P8_9 - K[2][2]*P_GF.P9_9;

    P_new.P4_4 = P_GF.P4_4 - K[3][0]*P_GF.P4_7 - K[3][1]*P_GF.P4_8 - K[3][2]*P_GF.P4_9;
    P_new.P4_5 = P_GF.P4_5 - K[3][0]*P_GF.P5_7 - K[3][1]*P_GF.P5_8 - K[3][2]*P_GF.P5_9;
    P_new.P4_6 = P_GF.P4_6 - K[3][0]*P_GF.P6_7 - K[3][1]*P_GF.P6_8 - K[3][2]*P_GF.P6_9;
    P_new.P4_7 = P_GF.P4_7 - K[3][0]*P_GF.P7_7 - K[3][1]*P_GF.P7_8 - K[3][2]*P_GF.P7_9;
    P_new.P4_8 = P_GF.P4_8 - K[3][0]*P_GF.P7_8 - K[3][1]*P_GF.P8_8 - K[3][2]*P_GF.P8_9;
    P_new.P4_9 = P_GF.P4_9 - K[3][0]*P_GF.P7_9 - K[3][1]*P_GF.P8_9 - K[3][2]*P_GF.P9_9;

    P_new.P5_5 = P_GF.P5_5 - K[4][0]*P_GF.P5_7 - K[4][1]*P_GF.P5_8 - K[4][2]*P_GF.P5_9;
    P_new.P5_6 = P_GF.P5_6 - K[4][0]*P_GF.P6_7 - K[4][1]*P_GF.P6_8 - K[4][2]*P_GF.P6_9;
    P_new.P5_7 = P_GF.P5_7 - K[4][0]*P_GF.P7_7 - K[4][1]*P_GF.P7_8 - K[4][2]*P_GF.P7_9;
    P_new.P5_8 = P_GF.P5_8 - K[4][0]*P_GF.P7_8 - K[4][1]*P_GF.P8_8 - K[4][2]*P_GF.P8_9;
    P_new.P5_9 = P_GF.P5_9 - K[4][0]*P_GF.P7_9 - K[4][1]*P_GF.P8_9 - K[4][2]*P_GF.P9_9;

    P_new.P6_6 = P_GF.P6_6 - K[5][0]*P_GF.P6_7 - K[5][1]*P_GF.P6_8 - K[5][2]*P_GF.P6_9;
    P_new.P6_7 = P_GF.P6_7 - K[5][0]*P_GF.P7_7 - K[5][1]*P_GF.P7_8 - K[5][2]*P_GF.P7_9;
    P_new.P6_8 = P_GF.P6_8 - K[5][0]*P_GF.P7_8 - K[5][1]*P_GF.P8_8 - K[5][2]*P_GF.P8_9;
    P_new.P6_9 = P_GF.P6_9 - K[5][0]*P_GF.P7_9 - K[5][1]*P_GF.P8_9 - K[5][2]*P_GF.P9_9;

    P_new.P7_7 = P_GF.P7_7 - K[6][0]*P_GF.P7_7 - K[6][1]*P_GF.P7_8 - K[6][2]*P_GF.P7_9;
    P_new.P7_8 = P_GF.P7_8 - K[6][0]*P_GF.P7_8 - K[6][1]*P_GF.P8_8 - K[6][2]*P_GF.P8_9;
    P_new.P7_9 = P_GF.P7_9 - K[6][0]*P_GF.P7_9 - K[6][1]*P_GF.P8_9 - K[6][2]*P_GF.P9_9;
    
    P_new.P8_8 = P_GF.P8_8 - K[7][0]*P_GF.P7_8 - K[7][1]*P_GF.P8_8 - K[7][2]*P_GF.P8_9;
    P_new.P8_9 = P_GF.P8_9 - K[7][0]*P_GF.P7_9 - K[7][1]*P_GF.P8_9 - K[7][2]*P_GF.P9_9;

    P_new.P9_9 = P_GF.P9_9 - K[8][0]*P_GF.P7_9 - K[8][1]*P_GF.P8_9 - K[8][2]*P_GF.P9_9;
    
    P_GF = P_new;
}

double Ground_Filter_data(uint8_t index){
    return Ground_Filter_xhat[index];
}

void Initialize_Air_Filter(){
    memset(Air_Filter_xhat, 0, sizeof(Air_Filter_xhat));
    memset(&P_AF, 0, sizeof(P_AF));

    // Seed velocity and drag coefficient uncertainties so that they will converge faster
    P_AF.P1_1 = 1.0;
    P_AF.P2_2 = 1.0;
    P_AF.P3_3 = 1.0;
    
    // The initial conditions for the Euler angle estimates will be taken from the ground filter states
    Air_Filter_xhat[3] = Ground_Filter_xhat[6];
    Air_Filter_xhat[4] = Ground_Filter_xhat[7];
    Air_Filter_xhat[5] = Ground_Filter_xhat[8];
    
    // Make an initial guess of the drag coefficient
    Air_Filter_xhat[2] = 0.5;
}

void Air_Filter_Predict(){
    Air_Filter_Covariance P_new;
    static double xdot_last[6] = {0};
    double xdot[6] = {0};

    double phi = Air_Filter_xhat[3];
    double theta = Air_Filter_xhat[4];
    // If there are gyro biases, remove them from inputs
    double p = IMU_Angular_Rate(0)*D2R - Ground_Filter_xhat[3];
    double q = IMU_Angular_Rate(1)*D2R - Ground_Filter_xhat[4];
    double r = IMU_Angular_Rate(2)*D2R - Ground_Filter_xhat[5];

    // Perform nonlinear prediction
    xdot[0] = -g_gravity*sin(theta) - (Air_Filter_xhat[2]*Air_Filter_xhat[0]/g_mass);
    xdot[1] = g_gravity*sin(phi)*cos(theta) - (Air_Filter_xhat[2]*Air_Filter_xhat[1]/g_mass);
    xdot[4] = cos(phi)*q - sin(phi)*r;
    if (abs(theta) != M_PI/2.0){
        xdot[3] = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
        xdot[5] = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    }
    // Integrate using trapezoidal rule
    for (uint8_t i = 0; i < 6; i++){
        Air_Filter_xhat[i] += 0.5*d_t*(xdot[i] + xdot_last[i]);
        xdot_last[i] = xdot[i];
    }
    
    // Update state transition matrix 
    double u = Air_Filter_xhat[0];
    double v = Air_Filter_xhat[1];
    double mu = Air_Filter_xhat[2];
    phi = Air_Filter_xhat[3];
    theta = Air_Filter_xhat[4];

    double c1 = 1.0 - d_t*(mu/g_mass);
    double c2 = -d_t*(u/g_mass);
    double c3 = -d_t*g_gravity*cos(theta);
    double c4 = -d_t*(v/g_mass);
    double c5 = d_t*g_gravity*cos(theta)*cos(phi);
    double c6 = -d_t*g_gravity*sin(theta)*sin(phi);
    
    P_new.P1_1 = P_AF.P1_1*pow(c1, 2) + 2.0*P_AF.P1_3*c1*c2 + 2.0*P_AF.P1_5*c1*c3 + P_AF.P3_3*pow(c2, 2)
                 + 2.0*P_AF.P3_5*c2*c3 + P_AF.P5_5*pow(c3, 2);
    P_new.P1_2 = c1*(P_AF.P1_2*c1 + P_AF.P2_3*c2 + P_AF.P2_5*c3) + c4*(P_AF.P1_3*c1 + P_AF.P3_3*c2 + P_AF.P3_5*c3) + 
                    c5*(P_AF.P1_4*c1 + P_AF.P3_4*c2 + P_AF.P4_5*c3) + c6*(P_AF.P1_5*c1 + P_AF.P3_5*c2 + P_AF.P5_5*c3);
    P_new.P1_3 = P_AF.P1_3*c1 + P_AF.P3_3*c2 + P_AF.P3_5*c3;
    P_new.P1_4 = P_AF.P1_4*c1 + P_AF.P3_4*c2 + P_AF.P4_5*c3;
    P_new.P1_5 = P_AF.P1_5*c1 + P_AF.P3_5*c2 + P_AF.P5_5*c3;
    P_new.P1_6 = P_AF.P1_6*c1 + P_AF.P3_6*c2 + P_AF.P5_6*c3;

    P_new.P2_2 = P_AF.P2_2*pow(c1, 2) + 2.0*P_AF.P2_3*c1*c4 + 2.0*P_AF.P2_4*c1*c5 + 2.0*P_AF.P2_5*c1*c6 + P_AF.P3_3*pow(c4, 2)
                    + 2.0*P_AF.P3_4*c4*c5 + 2.0*P_AF.P3_5*c4*c6 + P_AF.P4_4*pow(c5, 2) + 2.0*P_AF.P4_5*c5*c6 + P_AF.P5_5*pow(c6, 2);
    P_new.P2_3 = P_AF.P2_3*c1 + P_AF.P3_3*c4 + P_AF.P3_4*c5 + P_AF.P3_5*c6;
    P_new.P2_4 = P_AF.P2_4*c1 + P_AF.P3_4*c4 + P_AF.P4_4*c5 + P_AF.P4_5*c6;
    P_new.P2_5 = P_AF.P2_5*c1 + P_AF.P3_5*c4 + P_AF.P4_5*c5 + P_AF.P5_5*c6;
    P_new.P2_6 = P_AF.P2_6*c1 + P_AF.P3_6*c4 + P_AF.P4_6*c5 + P_AF.P5_6*c6;
    
    P_new.P3_3 = P_AF.P3_3;
    P_new.P3_4 = P_AF.P3_4;
    P_new.P3_5 = P_AF.P3_5;
    P_new.P3_6 = P_AF.P3_6;
    
    P_new.P4_4 = P_AF.P4_4 + A_Process_Cov.P4_4;
    P_new.P4_5 = P_AF.P4_5 + A_Process_Cov.P4_5;
    P_new.P4_6 = P_AF.P4_6 + A_Process_Cov.P4_6;
    
    P_new.P5_5 = P_AF.P5_5 + A_Process_Cov.P5_5;
    P_new.P5_6 = P_AF.P5_6 + A_Process_Cov.P5_6;
    
    P_new.P6_6 = P_AF.P6_6 + A_Process_Cov.P6_6;
    
    P_AF = P_new;
}

void Air_Filter_Update(){
    Matrix_3 S = {0};
    Matrix_3 S_inv = {0};
    double R[3] = {Accel_RMS_Noise*g_gravity, Accel_RMS_Noise*g_gravity, Mag_RMS_Noise};
    double measurement[3];
    double u = Air_Filter_xhat[0];
    double v = Air_Filter_xhat[1];
    double mu = Air_Filter_xhat[2];
    double phi = Air_Filter_xhat[3];
    double theta = Air_Filter_xhat[4];
    
    // Perform measurements

    measurement[0] = IMU_Acceleration(0)*g_gravity;
    measurement[1] = IMU_Acceleration(1)*g_gravity;
    
    double mag_x_NED = cos(theta)*Magnetometer_Field(0) + sin(phi)*sin(theta)*Magnetometer_Field(1) + cos(phi)*sin(theta)*Magnetometer_Field(2);
    double mag_y_NED = cos(phi)*Magnetometer_Field(1) - sin(phi)*Magnetometer_Field(2);
    
    measurement[2] = -atan2(mag_y_NED, mag_x_NED);
    if (isnan(measurement[2])){
        measurement[2] = Air_Filter_xhat[5];
    }
    
    double ybar[3] = {
        measurement[0] + (mu*u)/g_mass,
        measurement[1] + (mu*v)/g_mass,
        measurement[2] - Air_Filter_xhat[5]
    };
    
    // Update measurement matrix
    double c1 = -mu/g_mass;
    double c2 = -u/g_mass;
    double c3 = -v/g_mass;
    
    // Calculate innovation
    S.data[0][0] = P_AF.P1_1*pow(c1, 2) + 2.0*P_AF.P1_3*c1*c2 + P_AF.P3_3*pow(c2, 2) + R[0];
    S.data[0][1] = c1*(P_AF.P1_2*c1 + P_AF.P2_3*c2) + c3*(P_AF.P1_3*c1 + P_AF.P3_3*c2);
    S.data[0][2] = P_AF.P1_6*c1 + P_AF.P3_6*c2;
    S.data[1][0] = S.data[0][1];
    S.data[1][1] = P_AF.P2_2*pow(c1, 2) + 2.0*P_AF.P2_3*c1*c3 + P_AF.P3_3*pow(c3, 2) + R[1];
    S.data[1][2] = P_AF.P2_6*c1 + P_AF.P3_6*c3;
    S.data[2][0] = S.data[0][2];
    S.data[2][1] = S.data[1][2];
    S.data[2][2] = P_AF.P6_6 + R[2];
    
    // Invert and check that inverse was successful
    bool S_inv_successful = Mat3_Inv(&S, &S_inv);
    if (!S_inv_successful) return;
    
    // Calculate Kalman gain
    double K[6][3] =
        {{P_AF.P1_6*S_inv.data[2][0] + S_inv.data[0][0]*(P_AF.P1_1*c1 + P_AF.P1_3*c2) + S_inv.data[1][0]*(P_AF.P1_2*c1 + P_AF.P1_3*c3),
          P_AF.P1_6*S_inv.data[2][1] + S_inv.data[0][1]*(P_AF.P1_1*c1 + P_AF.P1_3*c2) + S_inv.data[1][1]*(P_AF.P1_2*c1 + P_AF.P1_3*c3),
          P_AF.P1_6*S_inv.data[2][2] + S_inv.data[0][2]*(P_AF.P1_1*c1 + P_AF.P1_3*c2) + S_inv.data[1][2]*(P_AF.P1_2*c1 + P_AF.P1_3*c3)},
          {P_AF.P2_6*S_inv.data[2][0] + S_inv.data[0][0]*(P_AF.P1_2*c1 + P_AF.P2_3*c2) + S_inv.data[1][0]*(P_AF.P2_2*c1 + P_AF.P2_3*c3),
          P_AF.P2_6*S_inv.data[2][1] + S_inv.data[0][1]*(P_AF.P1_2*c1 + P_AF.P2_3*c2) + S_inv.data[1][1]*(P_AF.P2_2*c1 + P_AF.P2_3*c3),
          P_AF.P2_6*S_inv.data[2][2] + S_inv.data[0][2]*(P_AF.P1_2*c1 + P_AF.P2_3*c2) + S_inv.data[1][2]*(P_AF.P2_2*c1 + P_AF.P2_3*c3)},
          {P_AF.P3_6*S_inv.data[2][0] + S_inv.data[0][0]*(P_AF.P1_3*c1 + P_AF.P3_3*c2) + S_inv.data[1][0]*(P_AF.P2_3*c1 + P_AF.P3_3*c3),
          P_AF.P3_6*S_inv.data[2][1] + S_inv.data[0][1]*(P_AF.P1_3*c1 + P_AF.P3_3*c2) + S_inv.data[1][1]*(P_AF.P2_3*c1 + P_AF.P3_3*c3),
          P_AF.P3_6*S_inv.data[2][2] + S_inv.data[0][2]*(P_AF.P1_3*c1 + P_AF.P3_3*c2) + S_inv.data[1][2]*(P_AF.P2_3*c1 + P_AF.P3_3*c3)},
          {P_AF.P4_6*S_inv.data[2][0] + S_inv.data[0][0]*(P_AF.P1_4*c1 + P_AF.P3_4*c2) + S_inv.data[1][0]*(P_AF.P2_4*c1 + P_AF.P3_4*c3),
          P_AF.P4_6*S_inv.data[2][1] + S_inv.data[0][1]*(P_AF.P1_4*c1 + P_AF.P3_4*c2) + S_inv.data[1][1]*(P_AF.P2_4*c1 + P_AF.P3_4*c3),
          P_AF.P4_6*S_inv.data[2][2] + S_inv.data[0][2]*(P_AF.P1_4*c1 + P_AF.P3_4*c2) + S_inv.data[1][2]*(P_AF.P2_4*c1 + P_AF.P3_4*c3)},
          {P_AF.P5_6*S_inv.data[2][0] + S_inv.data[0][0]*(P_AF.P1_5*c1 + P_AF.P3_5*c2) + S_inv.data[1][0]*(P_AF.P2_5*c1 + P_AF.P3_5*c3),
          P_AF.P5_6*S_inv.data[2][1] + S_inv.data[0][1]*(P_AF.P1_5*c1 + P_AF.P3_5*c2) + S_inv.data[1][1]*(P_AF.P2_5*c1 + P_AF.P3_5*c3),
          P_AF.P5_6*S_inv.data[2][2] + S_inv.data[0][2]*(P_AF.P1_5*c1 + P_AF.P3_5*c2) + S_inv.data[1][2]*(P_AF.P2_5*c1 + P_AF.P3_5*c3)},
          {P_AF.P6_6*S_inv.data[2][0] + S_inv.data[0][0]*(P_AF.P1_6*c1 + P_AF.P3_6*c2) + S_inv.data[1][0]*(P_AF.P2_6*c1 + P_AF.P3_6*c3),
          P_AF.P6_6*S_inv.data[2][1] + S_inv.data[0][1]*(P_AF.P1_6*c1 + P_AF.P3_6*c2) + S_inv.data[1][1]*(P_AF.P2_6*c1 + P_AF.P3_6*c3),
          P_AF.P6_6*S_inv.data[2][2] + S_inv.data[0][2]*(P_AF.P1_6*c1 + P_AF.P3_6*c2) + S_inv.data[1][2]*(P_AF.P2_6*c1 + P_AF.P3_6*c3)}};
    
    // Update state using measurement
    double x_new[6];
    for (uint8_t i = 0; i < 6; i++){
        x_new[i] = Air_Filter_xhat[i] + K[i][0]*ybar[0] + K[i][1]*ybar[1] + K[i][2]*ybar[2];
    }
    memcpy(Air_Filter_xhat, x_new, sizeof(x_new));

    Air_Filter_Covariance P_new;
    
    P_new.P1_1 = P_AF.P1_1*(1.0 - K[0][0]*c1) - P_AF.P1_3*c2*(K[0][0] + K[0][1]*c3) - K[0][2]*P_AF.P1_6 - K[0][1]*P_AF.P1_2*c3;
    P_new.P1_2 = P_AF.P1_2*(1.0 - K[0][0]*c1) - P_AF.P2_3*(K[0][0]*c2 + K[0][1]*c3) - K[0][2]*P_AF.P2_6 - K[0][1]*P_AF.P2_2*c1;
    P_new.P1_3 = P_AF.P1_3*(1.0 - K[0][0]*c1) - P_AF.P3_3*(K[0][0]*c2 + K[0][1]*c3) - K[0][2]*P_AF.P3_6 - K[0][1]*P_AF.P2_3*c1;
    P_new.P1_4 = P_AF.P1_4*(1.0 - K[0][0]*c1) - P_AF.P3_4*(K[0][0]*c2 + K[0][1]*c3) - K[0][2]*P_AF.P4_6 - K[0][1]*P_AF.P2_4*c1;
    P_new.P1_5 = P_AF.P1_5*(1.0 - K[0][0]*c1) - P_AF.P3_5*(K[0][0]*c2 + K[0][1]*c3) - K[0][2]*P_AF.P5_6 - K[0][1]*P_AF.P2_5*c1;
    P_new.P1_6 = P_AF.P1_6*(1.0 - K[0][0]*c1) - P_AF.P3_6*(K[0][0]*c2 + K[0][1]*c3) - K[0][2]*P_AF.P6_6 - K[0][1]*P_AF.P2_6*c1;

    P_new.P2_2 = P_AF.P2_2*(1.0 - K[1][1]*c1) - P_AF.P2_3*(K[1][0]*c2 + K[1][1]*c3) - K[1][2]*P_AF.P2_6 - K[1][0]*P_AF.P1_2*c1;
    P_new.P2_3 = P_AF.P2_3*(1.0 - K[1][1]*c1) - P_AF.P3_3*(K[1][0]*c2 + K[1][1]*c3) - K[1][2]*P_AF.P3_6 - K[1][0]*P_AF.P1_3*c1;
    P_new.P2_4 = P_AF.P2_4*(1.0 - K[1][1]*c1) - P_AF.P3_4*(K[1][0]*c2 + K[1][1]*c3) - K[1][2]*P_AF.P4_6 - K[1][0]*P_AF.P1_4*c1;
    P_new.P2_5 = P_AF.P2_5*(1.0 - K[1][1]*c1) - P_AF.P3_5*(K[1][0]*c2 + K[1][1]*c3) - K[1][2]*P_AF.P5_6 - K[1][0]*P_AF.P1_5*c1;
    P_new.P2_6 = P_AF.P2_6*(1.0 - K[1][1]*c1) - P_AF.P3_6*(K[1][0]*c2 + K[1][1]*c3) - K[1][2]*P_AF.P6_6 - K[1][0]*P_AF.P1_6*c1;
    
    P_new.P3_3 = P_AF.P3_3*(1.0 - K[2][0]*c2 - K[2][1]*c3) - K[2][2]*P_AF.P3_6 - K[2][0]*P_AF.P1_3*c1 - K[2][1]*P_AF.P2_3*c1;
    P_new.P3_4 = P_AF.P3_4*(1.0 - K[2][0]*c2 - K[2][1]*c3) - K[2][2]*P_AF.P4_6 - K[2][0]*P_AF.P1_4*c1 - K[2][1]*P_AF.P2_4*c1;
    P_new.P3_5 = P_AF.P3_5*(1.0 - K[2][0]*c2 - K[2][1]*c3) - K[2][2]*P_AF.P5_6 - K[2][0]*P_AF.P1_5*c1 - K[2][1]*P_AF.P2_5*c1;
    P_new.P3_6 = P_AF.P3_6*(1.0 - K[2][0]*c2 - K[2][1]*c3) - K[2][2]*P_AF.P6_6 - K[2][0]*P_AF.P1_6*c1 - K[2][1]*P_AF.P2_6*c1;
    
    P_new.P4_4 = P_AF.P4_4 - P_AF.P3_4*(K[3][0]*c2 + K[3][1]*c3) - K[3][2]*P_AF.P4_6 - K[3][0]*P_AF.P1_4*c1 - K[3][1]*P_AF.P2_4*c1;
    P_new.P4_5 = P_AF.P4_5 - P_AF.P3_5*(K[3][0]*c2 + K[3][1]*c3) - K[3][2]*P_AF.P5_6 - K[3][0]*P_AF.P1_5*c1 - K[3][1]*P_AF.P2_5*c1;
    P_new.P4_6 = P_AF.P4_6 - P_AF.P3_6*(K[3][0]*c2 + K[3][1]*c3) - K[3][2]*P_AF.P6_6 - K[3][0]*P_AF.P1_6*c1 - K[3][1]*P_AF.P2_6*c1;
    
    P_new.P5_5 = P_AF.P5_5 - P_AF.P3_5*(K[4][0]*c2 + K[4][1]*c3) - K[4][2]*P_AF.P5_6 - K[4][0]*P_AF.P1_5*c1 - K[4][1]*P_AF.P2_5*c1;
    P_new.P5_6 = P_AF.P5_6 - P_AF.P3_6*(K[4][0]*c2 + K[4][1]*c3) - K[4][2]*P_AF.P6_6 - K[4][0]*P_AF.P1_6*c1 - K[4][1]*P_AF.P2_6*c1;
    
    P_new.P6_6 = P_AF.P6_6*(1.0 - K[5][2]) - P_AF.P3_6*(K[5][0]*c2 + K[5][1]*c3) - K[5][0]*P_AF.P1_6*c1 - K[5][1]*P_AF.P2_6*c1;
    
    P_AF = P_new;
}