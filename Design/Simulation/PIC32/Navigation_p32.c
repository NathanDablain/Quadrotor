#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "Navigation_p32.h"
#include "Kalman_Filter_p32.h"
#include "IMU_p32.h"
#include "Magnetometer_p32.h"
#include "Barometer_p32.h"
#include "Sim_Types.h"
#include "Global_Variables_p32.h"
#include "External_Interface.h"
#include "Guidance_p32.h"

// Flag to control whether to use the stack implementation of the kalman filters or the heap implementation
static const bool Use_Heap_All = false;
static const bool Use_Heap_Gnd = false;
static const bool Use_Heap_Air = false;
static const bool Use_Heap_Alt = false;
// QuadrotorsandAccelerometers.pdf
static Kalman_Filter *Ground_Filter;
static Kalman_Filter *Air_Filter;
static Kalman_Filter *Altitude_Filter;
// In units of Hz
static const double Gyro_ODR_2 = 20.0;//57.706152185014034;
static const double Accel_ODR_2 = 20.0;//40.743097574926725;
// Date sheet gives value of 5mdps/sqrt(ODR), convert to rad/s
static const double Gyro_RMS_Noise = 0.005*Gyro_ODR_2*D2R;
// Data sheet gives value of 60ug/sqrt(ODR), convert to g
static const double Accel_RMS_Noise = 0.00006*Accel_ODR_2;
// Assume drift rate of 20 deg/hr, convert to rad/s
const double Gyro_Bias_Instability = (20.0/3600.0)*D2R;
// Constant value if LPF = ODR/2
const double Mag_RMS_Noise = 4.5/500.0;
// Given the ODR is 75Hz and the LPF setting is ODR/20, 0.65 in Pa which at STP corresponds to 0.054 m
const double BAR_RMS_Noise = 0.054;

// Holds ground filter estimated states, w_bias, w, Euler
static double Ground_Filter_xhat[9];
// Holds air filter estimated states, u, v, mu, phi, theta, psi
static double Air_Filter_xhat[6];
static double Air_Filter_xdot_last[6];
static double Air_Filter_xdot_last2[6];
static double Air_Filter_xdot[6];
// Holds Altitude filter estimated states, h, h_dot
static double Altitude_Filter_xhat[2];
static double Altitude_Filter_xdot_last[2];

static Ground_Filter_Covariance P_GF;
static Air_Filter_Covariance P_AF;

void Run_Ground_Filter(bool Initialize){
    const Time Filter_Predict_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/200};
    static Time Filter_Predict_Last;
    const Time Filter_Update_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/100};
    static Time Filter_Update_Last;

    if (Initialize){
        if (Use_Heap_Gnd || Use_Heap_All){
            if (Ground_Filter != NULL) Filter_Destructor(Ground_Filter, 7);
                Ground_Filter = Filter_Constructor(9, 3, 3);
                assert(Ground_Filter != NULL);

                Ground_Filter->F->data[0][3] = -1.0;
                Ground_Filter->F->data[1][4] = -1.0;
                Ground_Filter->F->data[2][5] = -1.0;
                Ground_Filter->F->data[3][3] = 1.0;
                Ground_Filter->F->data[4][4] = 1.0;
                Ground_Filter->F->data[5][5] = 1.0;
                Ground_Filter->F->data[6][6] = 1.0;
                Ground_Filter->F->data[7][7] = 1.0;
                Ground_Filter->F->data[8][8] = 1.0;

                Ground_Filter->H->data[0][6] = 1.0;
                Ground_Filter->H->data[1][7] = 1.0;
                Ground_Filter->H->data[2][8] = 1.0;

                Ground_Filter->B->data[0][0] = 1.0;
                Ground_Filter->B->data[1][1] = 1.0;
                Ground_Filter->B->data[2][2] = 1.0;

                // Seed bias entries in covariance matrix to reduce estimation time of initial offset
                Ground_Filter->P->data[3][3] = 1.0;
                Ground_Filter->P->data[4][4] = 1.0;
                Ground_Filter->P->data[5][5] = 1.0;
                Ground_Filter->P->data[6][6] = 1.0;
                Ground_Filter->P->data[7][7] = 1.0;
                Ground_Filter->P->data[8][8] = 1.0;
                double d_t = 1.0/200.0;//Gyro_ODR;

                Ground_Filter->Q->data[0][0] = Gyro_RMS_Noise;
                Ground_Filter->Q->data[1][1] = Gyro_RMS_Noise;
                Ground_Filter->Q->data[2][2] = Gyro_RMS_Noise;
                Ground_Filter->Q->data[3][3] = Gyro_Bias_Instability;
                Ground_Filter->Q->data[4][4] = Gyro_Bias_Instability;
                Ground_Filter->Q->data[5][5] = Gyro_Bias_Instability;
                Ground_Filter->Q->data[6][6] = Gyro_RMS_Noise * d_t;
                Ground_Filter->Q->data[7][7] = Gyro_RMS_Noise * d_t;
                Ground_Filter->Q->data[8][8] = Gyro_RMS_Noise * d_t;

                Ground_Filter->R->data[0][0] = Accel_RMS_Noise;
                Ground_Filter->R->data[1][1] = Accel_RMS_Noise;
                Ground_Filter->R->data[2][2] = Mag_RMS_Noise;
        }
        else{
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

        Filter_Predict_Last = Current_Time();
        Filter_Update_Last = Current_Time();
    }
    else{
        if (Compare_And_Update(Current_Time(), Filter_Predict_Rate, &Filter_Predict_Last)){
            if (Use_Heap_Gnd || Use_Heap_All){
                Ground_Filter_Predict();
            }
            else{
                Ground_Filter_Predict_p32();
            }
        }
        if (Compare_And_Update(Current_Time(), Filter_Update_Rate, &Filter_Update_Last)){
            if (Use_Heap_Gnd || Use_Heap_All){
                Ground_Filter_Update();
            }
            else{
                Ground_Filter_Update_p32();
            }
        }
    }
}

void Run_Air_Filter(bool Initialize){
    const Time Filter_Predict_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/200};
    static Time Filter_Predict_Last;
    const Time Filter_Update_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/50};
    static Time Filter_Update_Last;
    bool Inhibit_Update = false;
    
    if (Initialize){
        // The accelerometer low pass filter was initially set with a BW of 1660/400 -> 4 Hz
        // We need to raise this bandwidth above our filter update frequency of 50 Hz
        // Set to 1660/10 -> 166 Hz
        e_accel_lpf_setting = 0;
        // The gyro BW was set to 153 Hz, disable lpf for flight 
        e_gyro_lpf_setting = 4;
        e_imu_settings_updated = true;
        if (Use_Heap_Air || Use_Heap_All){
            if (Air_Filter != NULL) Filter_Destructor(Air_Filter, 7);
            Air_Filter = Filter_Constructor(6, 3, 3);
            if (Air_Filter == NULL) return;

            Air_Filter->F->data[2][2] = 1.0;
            Air_Filter->F->data[3][3] = 1.0;
            Air_Filter->F->data[4][4] = 1.0;
            Air_Filter->F->data[5][5] = 1.0;

            Air_Filter->H->data[2][5] = 1.0;
            
            // Initialize process, state, and system covariances
            Air_Filter->P->data[2][2] = 0.1;
            Air_Filter->xhat->data[2][0] = 0.1;
            double d_t = 1.0/200.0;//Gyro_ODR;

            // Grab initial euler angles from ground filter
            if (Use_Heap_Gnd || Use_Heap_All){
                Air_Filter->xhat->data[3][0] = Ground_Filter->xhat->data[6][0];
                Air_Filter->xhat->data[4][0] = Ground_Filter->xhat->data[7][0];
                Air_Filter->xhat->data[5][0] = Ground_Filter->xhat->data[8][0];
            }
            else{
                Air_Filter->xhat->data[3][0] = Ground_Filter_xhat[6];
                Air_Filter->xhat->data[4][0] = Ground_Filter_xhat[7];
                Air_Filter->xhat->data[5][0] = Ground_Filter_xhat[8];
            }

            Air_Filter->Q->data[3][3] = 0.1*Gyro_RMS_Noise*d_t;
            Air_Filter->Q->data[4][4] = 0.1*Gyro_RMS_Noise*d_t;
            Air_Filter->Q->data[5][5] = 0.1*Gyro_RMS_Noise*d_t;

            Air_Filter->R->data[0][0] = Accel_RMS_Noise*g_gravity;
            Air_Filter->R->data[1][1] = Accel_RMS_Noise*g_gravity;
            Air_Filter->R->data[2][2] = Mag_RMS_Noise;
        }
        else {
            memset(Air_Filter_xhat, 0, sizeof(Air_Filter_xhat));
            memset(&P_AF, 0, sizeof(P_AF));

            // Seed drag coefficient uncertainty so that it will converge faster
            P_AF.P3_3 = 0.1;

            // The initial conditions for the Euler angle estimates will be taken from the ground filter states
            if (Use_Heap_Gnd){
                Air_Filter_xhat[3] = Ground_Filter->xhat->data[6][0];
                Air_Filter_xhat[4] = Ground_Filter->xhat->data[7][0];
                Air_Filter_xhat[5] = Ground_Filter->xhat->data[8][0];
            }
            else{
                Air_Filter_xhat[3] = Ground_Filter_xhat[6];
                Air_Filter_xhat[4] = Ground_Filter_xhat[7];
                Air_Filter_xhat[5] = Ground_Filter_xhat[8];
            }
            // Make an initial guess of the drag coefficient
            Air_Filter_xhat[2] = 0.1;
        }
        memset(Air_Filter_xdot_last, 0, sizeof(Air_Filter_xdot_last));
        memset(Air_Filter_xdot_last2, 0, sizeof(Air_Filter_xdot_last2));
        memset(Air_Filter_xdot, 0, sizeof(Air_Filter_xdot));
        Filter_Predict_Last = Current_Time();
        Filter_Update_Last = Current_Time();

    }
    else{
        if (Get_Guidance_State() < Climbing){
            Inhibit_Update = true;
        }

        if (Compare_And_Update(Current_Time(), Filter_Predict_Rate, &Filter_Predict_Last)){
            if (Use_Heap_Air || Use_Heap_All){
                Air_Filter_Predict(Inhibit_Update);
            }
            else {
                Air_Filter_Predict_p32(Inhibit_Update);
            }
        }
        // Run filter open loop when near the ground
        if (Compare_And_Update(Current_Time(), Filter_Update_Rate, &Filter_Update_Last) && !Inhibit_Update){
            if (Use_Heap_Air || Use_Heap_All){
                Air_Filter_Update();
            }
            else {
                Air_Filter_Update_p32();
            }
        }
    }
}

void Run_Altitude_Filter(bool Initialize){
    // This filter estimates the drones height off the ground as well as the velocity
    // in the same direction. It takes accelerometer outputs as inputs and barometer
    // outputs as measurements
    const Time Filter_Predict_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/200};
    static Time Filter_Predict_Last;
    const Time Filter_Update_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/75};
    static Time Filter_Update_Last;
    
    if (Initialize){
        if (Use_Heap_Alt || Use_Heap_All){
            if (Altitude_Filter != NULL) Filter_Destructor(Altitude_Filter, 7);
            Altitude_Filter = Filter_Constructor(2, 1, 1);
            if (Altitude_Filter == NULL) return;

            Altitude_Filter->F->data[0][0] = 1.0;
            Altitude_Filter->F->data[0][1] = 1.0/200.0;
            Altitude_Filter->F->data[1][1] = 1.0;

            Altitude_Filter->B->data[1][0] = 1.0/200.0;

            Altitude_Filter->H->data[0][0] = 1.0;

            Altitude_Filter->Q->data[1][1] = 0.001*Accel_RMS_Noise*g_gravity*(1.0/200.0);

            Altitude_Filter->R->data[0][0] = 0.1*BAR_RMS_Noise;
        }
        else {
            memset(Altitude_Filter_xhat, 0, sizeof(Altitude_Filter_xhat));
        }
        memset(Altitude_Filter_xdot_last, 0, sizeof(Altitude_Filter_xdot_last));

        Filter_Predict_Last = Current_Time();
        Filter_Update_Last = Current_Time();

    }
    else{
        if (Compare_And_Update(Current_Time(), Filter_Predict_Rate, &Filter_Predict_Last)){
            if (Use_Heap_Alt || Use_Heap_All){
                Altitude_Filter_Predict();
            }
            else {
                Altitude_Filter_Predict_p32();
            }
        }
        if (Compare_And_Update(Current_Time(), Filter_Update_Rate, &Filter_Update_Last)){
            if (Use_Heap_Alt || Use_Heap_All){
                Altitude_Filter_Update();
                // printf("%f   %f\n", Altitude_Filter->K->data[0][0], Altitude_Filter->K->data[1][0]);
            }
            else {
                Altitude_Filter_Update_p32();
            }
        }
    }
}

// Before take off, assume that normal forces acting on the drone enable measuring
// of the inverse g_gravity vector through these normal forces
// An initial kalman filter will be run under these assumptions in order to estimate
// initial attitude conditions and gyro bias instability
void Ground_Filter_Predict(){
    // States are : w_x, w_y, w_z, w_x_bias, w_y_bias, w_z_bias, phi, theta, psi
    // Inputs are : p, q, r
    // Measurements are: phi, theta, psi

    Matrix *input = Mat_Constructor(3, 1);

    // Update state transition matrix based on updated states
    Ground_Filter_State_Transition();

    // Perform prediction
    input->data[0][0] = IMU_Angular_Rate(0)*D2R;
    input->data[1][0] = IMU_Angular_Rate(1)*D2R;
    input->data[2][0] = IMU_Angular_Rate(2)*D2R;

    bool KF_Prediction_Status = Predict(Ground_Filter, input);
    assert(KF_Prediction_Status);

    Mat_Destructor(input);
}

void Ground_Filter_Predict_p32(){
    const double d_t = 1.0/200.0;//Gyro_ODR;
    const double Process_Noise[9] = {Gyro_RMS_Noise, Gyro_RMS_Noise, Gyro_RMS_Noise,
                                     Gyro_Bias_Instability, Gyro_Bias_Instability, Gyro_Bias_Instability,
                                     Gyro_RMS_Noise * d_t, Gyro_RMS_Noise * d_t, Gyro_RMS_Noise * d_t};
    double x_new[9] = {0};
    Ground_Filter_Covariance P_new;
    double s_phi = sin(Ground_Filter_xhat[6]);
    double c_phi = cos(Ground_Filter_xhat[6]);
    // Propagate state estimates
    x_new[0] = IMU_Angular_Rate(0)*D2R - Ground_Filter_xhat[3];
    x_new[1] = IMU_Angular_Rate(1)*D2R - Ground_Filter_xhat[4];
    x_new[2] = IMU_Angular_Rate(2)*D2R - Ground_Filter_xhat[5];
    x_new[3] = Ground_Filter_xhat[3];
    x_new[4] = Ground_Filter_xhat[4];
    x_new[5] = Ground_Filter_xhat[5];
    x_new[7] = Ground_Filter_xhat[7] + d_t*(Ground_Filter_xhat[1]*c_phi - Ground_Filter_xhat[2]*s_phi);
    if (fabs(Ground_Filter_xhat[8]) != M_PI/2.0){
        double t_theta = tan(Ground_Filter_xhat[7]);
        double c_theta = cos(Ground_Filter_xhat[7]);
        x_new[6] = Ground_Filter_xhat[6] + d_t*(Ground_Filter_xhat[0] + Ground_Filter_xhat[1]*s_phi*t_theta + Ground_Filter_xhat[2]*c_phi*t_theta); 
        x_new[8] = Ground_Filter_xhat[8] + d_t*((Ground_Filter_xhat[1]*s_phi/c_theta) + (Ground_Filter_xhat[2]*c_phi/c_theta)); 
        
        // Propagate covariance estimates
        double c1 = d_t;
        double c2 = d_t*s_phi*t_theta;
        double c3 = d_t*c_phi*t_theta;
        double c4 = d_t*c_phi;
        double c5 = -d_t*s_phi;
        double c6 = d_t*s_phi/c_theta;
        double c7 = d_t*c_phi/c_theta;

        P_new.P1_1 = P_GF.P4_4 + Process_Noise[0];
        P_new.P1_2 = P_GF.P4_5;
        P_new.P1_3 = P_GF.P4_6;
        P_new.P1_4 = -P_GF.P4_4;
        P_new.P1_5 = -P_GF.P4_5;
        P_new.P1_6 = -P_GF.P4_6;
        P_new.P1_7 = -P_GF.P1_4*c1 - P_GF.P2_4*c2 - P_GF.P3_4*c3 - P_GF.P4_7;
        P_new.P1_8 = -P_GF.P2_4*c4 - P_GF.P3_4*c5 - P_GF.P4_8;
        P_new.P1_9 = -P_GF.P2_4*c6 - P_GF.P3_4*c7 - P_GF.P4_9;

        P_new.P2_2 = P_GF.P5_5 + Process_Noise[1];
        P_new.P2_3 = P_GF.P5_6;
        P_new.P2_4 = -P_GF.P4_5;
        P_new.P2_5 = -P_GF.P5_5;
        P_new.P2_6 = -P_GF.P5_6;
        P_new.P2_7 = -P_GF.P1_5*c1 - P_GF.P2_5*c2 - P_GF.P3_5*c3 - P_GF.P5_7;
        P_new.P2_8 = -P_GF.P2_5*c4 - P_GF.P3_5*c5 - P_GF.P5_8;
        P_new.P2_9 = -P_GF.P2_5*c6 - P_GF.P3_5*c7 - P_GF.P5_9;

        P_new.P3_3 = P_GF.P6_6 + Process_Noise[2];
        P_new.P3_4 = -P_GF.P4_6;
        P_new.P3_5 = -P_GF.P5_6;
        P_new.P3_6 = -P_GF.P6_6;
        P_new.P3_7 = -P_GF.P1_6*c1 - P_GF.P2_6*c2 - P_GF.P3_6*c3 - P_GF.P6_7;
        P_new.P3_8 = -P_GF.P2_6*c4 - P_GF.P3_6*c5 - P_GF.P6_8;
        P_new.P3_9 = -P_GF.P2_6*c6 - P_GF.P3_6*c7 - P_GF.P6_9;

        P_new.P4_4 = P_GF.P4_4 + Process_Noise[3];
        P_new.P4_5 = P_GF.P4_5;
        P_new.P4_6 = P_GF.P4_6;
        P_new.P4_7 = P_GF.P1_4*c1 + P_GF.P2_4*c2 + P_GF.P3_4*c3 + P_GF.P4_7;
        P_new.P4_8 = P_GF.P2_4*c4 + P_GF.P3_4*c5 + P_GF.P4_8;
        P_new.P4_9 = P_GF.P2_4*c6 + P_GF.P3_4*c7 + P_GF.P4_9;

        P_new.P5_5 = P_GF.P5_5 + Process_Noise[4];
        P_new.P5_6 = P_GF.P5_6;
        P_new.P5_7 = P_GF.P1_5*c1 + P_GF.P2_5*c2 + P_GF.P3_5*c3 + P_GF.P5_7;
        P_new.P5_8 = P_GF.P2_5*c4 + P_GF.P3_5*c5 + P_GF.P5_8;
        P_new.P5_9 = P_GF.P2_5*c6 + P_GF.P3_5*c7 + P_GF.P5_9;

        P_new.P6_6 = P_GF.P6_6 + Process_Noise[5];
        P_new.P6_7 = P_GF.P1_6*c1 + P_GF.P2_6*c2 + P_GF.P3_6*c3 + P_GF.P6_7;
        P_new.P6_8 = P_GF.P2_6*c4 + P_GF.P3_6*c5 + P_GF.P6_8;
        P_new.P6_9 = P_GF.P2_6*c6 + P_GF.P3_6*c7 + P_GF.P6_9;

        P_new.P7_7 = P_GF.P1_7*c1 + P_GF.P2_7*c2 + P_GF.P3_7*c3 + P_GF.P7_7 + c1*(P_GF.P1_1*c1 + P_GF.P1_2*c2 + P_GF.P1_3*c3 + P_GF.P1_7)
        + c2*(P_GF.P1_2*c1 + P_GF.P2_2*c2 + P_GF.P2_3*c3 + P_GF.P2_7) + c3*(P_GF.P1_3*c1 + P_GF.P2_3*c2 + P_GF.P3_3*c3 + P_GF.P3_7) + Process_Noise[6];
        P_new.P7_8 = P_GF.P2_7*c4 + P_GF.P3_7*c5 + P_GF.P7_8 + c1*(P_GF.P1_2*c4 + P_GF.P1_3*c5 + P_GF.P1_8)
        + c2*(P_GF.P2_2*c4 + P_GF.P2_3*c5 + P_GF.P2_8) + c3*(P_GF.P2_3*c4 + P_GF.P3_3*c5 + P_GF.P3_8);
        P_new.P7_9 = P_GF.P2_7*c6 + P_GF.P3_7*c7 + P_GF.P7_9 + c1*(P_GF.P1_2*c6 + P_GF.P1_3*c7 + P_GF.P1_9)
        + c2*(P_GF.P2_2*c6 + P_GF.P2_3*c7 + P_GF.P2_9) + c3*(P_GF.P2_3*c6 + P_GF.P3_3*c7 + P_GF.P3_9);

        P_new.P8_8 = P_GF.P2_8*c4 + P_GF.P3_8*c5 + P_GF.P8_8 + c4*(P_GF.P2_2*c4 + P_GF.P2_3*c5 + P_GF.P2_8) + c5*(P_GF.P2_3*c4 + P_GF.P3_3*c5 + P_GF.P3_8) + Process_Noise[7];
        P_new.P8_9 = P_GF.P2_8*c6 + P_GF.P3_8*c7 + P_GF.P8_9 + c4*(P_GF.P2_2*c6 + P_GF.P2_3*c7 + P_GF.P2_9) + c5*(P_GF.P2_3*c6 + P_GF.P3_3*c7 + P_GF.P3_9);

        P_new.P9_9 = P_GF.P2_9*c6 + P_GF.P3_9*c7 + P_GF.P9_9 + c6*(P_GF.P2_2*c6 + P_GF.P2_3*c7 + P_GF.P2_9) + c7*(P_GF.P2_3*c6 + P_GF.P3_3*c7 + P_GF.P3_9) + Process_Noise[8];

        P_GF = P_new;
    }
    memcpy(Ground_Filter_xhat, x_new, sizeof(x_new));
}

void Ground_Filter_Update(){
    Matrix *measurement = Mat_Constructor(3, 1);

    // Perform roll-pitch-yaw measurements
    double phi = Ground_Filter->xhat->data[6][0];
    double theta = Ground_Filter->xhat->data[7][0];
    measurement->data[0][0] = atan2((-IMU_Acceleration(1)), (-IMU_Acceleration(2)));
    if (isnan( measurement->data[0][0])){
        measurement->data[0][0] = phi;
    }
    measurement->data[1][0] = atan2((IMU_Acceleration(0)), sqrt(pow(IMU_Acceleration(1), 2) + pow(IMU_Acceleration(2), 2)));
    if (isnan(measurement->data[1][0])){
        measurement->data[1][0] = theta;
    }

    double mag_x_NED = cos(theta)*Magnetometer_Filtered_Field(0) + sin(phi)*sin(theta)*Magnetometer_Filtered_Field(1) + cos(phi)*sin(theta)*Magnetometer_Filtered_Field(2);
    double mag_y_NED = cos(phi)*Magnetometer_Filtered_Field(1) - sin(phi)*Magnetometer_Filtered_Field(2);
    measurement->data[2][0] = -atan2(mag_y_NED, mag_x_NED);
    if (isnan(measurement->data[2][0])){
        measurement->data[2][0] = Ground_Filter->xhat->data[8][0];
    }

    // Update state and covariances with measurement
    Update(Ground_Filter, measurement);

    Mat_Destructor(measurement);
}

void Ground_Filter_Update_p32(){
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

    double mag_x_NED = cos(theta)*Magnetometer_Filtered_Field(0) + s_phi*sin(theta)*Magnetometer_Filtered_Field(1) + c_phi*sin(theta)*Magnetometer_Filtered_Field(2);
    double mag_y_NED = c_phi*Magnetometer_Filtered_Field(1) - s_phi*Magnetometer_Filtered_Field(2);
    
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
    double K[9][3] = {{P_GF.P1_7*S_inv.data[0][0] + P_GF.P1_8*S_inv.data[1][0] + P_GF.P1_9*S_inv.data[2][0],
                       P_GF.P1_7*S_inv.data[0][1] + P_GF.P1_8*S_inv.data[1][1] + P_GF.P1_9*S_inv.data[2][1],
                       P_GF.P1_7*S_inv.data[0][2] + P_GF.P1_8*S_inv.data[1][2] + P_GF.P1_9*S_inv.data[2][2]},

                      {P_GF.P2_7*S_inv.data[0][0] + P_GF.P2_8*S_inv.data[1][0] + P_GF.P2_9*S_inv.data[2][0],
                       P_GF.P2_7*S_inv.data[0][1] + P_GF.P2_8*S_inv.data[1][1] + P_GF.P2_9*S_inv.data[2][1],
                       P_GF.P2_7*S_inv.data[0][2] + P_GF.P2_8*S_inv.data[1][2] + P_GF.P2_9*S_inv.data[2][2]},

                      {P_GF.P3_7*S_inv.data[0][0] + P_GF.P3_8*S_inv.data[1][0] + P_GF.P3_9*S_inv.data[2][0],
                       P_GF.P3_7*S_inv.data[0][1] + P_GF.P3_8*S_inv.data[1][1] + P_GF.P3_9*S_inv.data[2][1],
                       P_GF.P3_7*S_inv.data[0][2] + P_GF.P3_8*S_inv.data[1][2] + P_GF.P3_9*S_inv.data[2][2]},

                      {P_GF.P4_7*S_inv.data[0][0] + P_GF.P4_8*S_inv.data[1][0] + P_GF.P4_9*S_inv.data[2][0],
                       P_GF.P4_7*S_inv.data[0][1] + P_GF.P4_8*S_inv.data[1][1] + P_GF.P4_9*S_inv.data[2][1],
                       P_GF.P4_7*S_inv.data[0][2] + P_GF.P4_8*S_inv.data[1][2] + P_GF.P4_9*S_inv.data[2][2]},

                      {P_GF.P5_7*S_inv.data[0][0] + P_GF.P5_8*S_inv.data[1][0] + P_GF.P5_9*S_inv.data[2][0],
                       P_GF.P5_7*S_inv.data[0][1] + P_GF.P5_8*S_inv.data[1][1] + P_GF.P5_9*S_inv.data[2][1],
                       P_GF.P5_7*S_inv.data[0][2] + P_GF.P5_8*S_inv.data[1][2] + P_GF.P5_9*S_inv.data[2][2]},

                      {P_GF.P6_7*S_inv.data[0][0] + P_GF.P6_8*S_inv.data[1][0] + P_GF.P6_9*S_inv.data[2][0],
                       P_GF.P6_7*S_inv.data[0][1] + P_GF.P6_8*S_inv.data[1][1] + P_GF.P6_9*S_inv.data[2][1],
                       P_GF.P6_7*S_inv.data[0][2] + P_GF.P6_8*S_inv.data[1][2] + P_GF.P6_9*S_inv.data[2][2]},

                      {P_GF.P7_7*S_inv.data[0][0] + P_GF.P7_8*S_inv.data[1][0] + P_GF.P7_9*S_inv.data[2][0],
                       P_GF.P7_7*S_inv.data[0][1] + P_GF.P7_8*S_inv.data[1][1] + P_GF.P7_9*S_inv.data[2][1],
                       P_GF.P7_7*S_inv.data[0][2] + P_GF.P7_8*S_inv.data[1][2] + P_GF.P7_9*S_inv.data[2][2]},

                      {P_GF.P7_8*S_inv.data[0][0] + P_GF.P8_8*S_inv.data[1][0] + P_GF.P8_9*S_inv.data[2][0],
                       P_GF.P7_8*S_inv.data[0][1] + P_GF.P8_8*S_inv.data[1][1] + P_GF.P8_9*S_inv.data[2][1],
                       P_GF.P7_8*S_inv.data[0][2] + P_GF.P8_8*S_inv.data[1][2] + P_GF.P8_9*S_inv.data[2][2]},

                      {P_GF.P7_9*S_inv.data[0][0] + P_GF.P8_9*S_inv.data[1][0] + P_GF.P9_9*S_inv.data[2][0],
                       P_GF.P7_9*S_inv.data[0][1] + P_GF.P8_9*S_inv.data[1][1] + P_GF.P9_9*S_inv.data[2][1],
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

void Ground_Filter_State_Transition(){
    double d_t = 1.0/200.0;//Gyro_ODR;
    double phi = Ground_Filter->xhat->data[6][0];
    double theta = Ground_Filter->xhat->data[7][0];
    Ground_Filter->F->data[6][0] = d_t;
    Ground_Filter->F->data[7][1] = d_t*cos(phi);
    Ground_Filter->F->data[7][2] = -d_t*sin(phi);
    if (abs(theta) != M_PI/2.0){
        Ground_Filter->F->data[6][1] = d_t*sin(phi)*tan(theta);
        Ground_Filter->F->data[6][2] = d_t*cos(phi)*tan(theta);
        Ground_Filter->F->data[8][1] = d_t*sin(phi)/cos(theta);
        Ground_Filter->F->data[8][2] = d_t*cos(phi)/cos(theta);
    }
}

double Ground_Filter_data(uint8_t index){
    if (Use_Heap_Gnd || Use_Heap_All){
        return Ground_Filter->xhat->data[index][0];
    }
    else{
        return Ground_Filter_xhat[index];
    }
}

// After take off, assume that rotor drag force in the body x and y directions
// is proportional to the product of the body translational rate and some drag constant mu
// the accelerometers on the body x and y axes will measure this force and can be used to estimate 
// mu, roll, and pitch
void Air_Filter_Predict(bool Inhibit_Update){
    // States are : u, v, mu, phi, theta, psi
    // Inputs are : p, q, r
    // Measurements are: (mu/m)*u, (mu/m)*v, psi
    double d_t = 1.0/200.0;//Gyro_ODR;
    double phi = Air_Filter->xhat->data[3][0];
    double theta = Air_Filter->xhat->data[4][0];
    double p, q, r;
    if (Use_Heap_Gnd  || Use_Heap_All){
        p = IMU_Angular_Rate(0)*D2R - Ground_Filter->xhat->data[3][0];
        q = IMU_Angular_Rate(1)*D2R - Ground_Filter->xhat->data[4][0];
        r = IMU_Angular_Rate(2)*D2R - Ground_Filter->xhat->data[5][0];
    }
    else{
        p = IMU_Angular_Rate(0)*D2R - Ground_Filter_xhat[3];
        q = IMU_Angular_Rate(1)*D2R - Ground_Filter_xhat[4];
        r = IMU_Angular_Rate(2)*D2R - Ground_Filter_xhat[5];
    }

    if (Air_Filter->xhat->data[2][0] < MIN_MU){
        Air_Filter->xhat->data[2][0] = MIN_MU;
    }
    // Perform nonlinear prediction
    Air_Filter_xdot[0] = -g_gravity*sin(theta) - (Air_Filter->xhat->data[2][0]*Air_Filter->xhat->data[0][0]/g_mass);
    Air_Filter_xdot[1] = g_gravity*sin(phi)*cos(theta) - (Air_Filter->xhat->data[2][0]*Air_Filter->xhat->data[1][0]/g_mass);
    Air_Filter_xdot[4] = cos(phi)*q - sin(phi)*r;
    if (abs(theta) != M_PI/2.0){
        Air_Filter_xdot[3] = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
        Air_Filter_xdot[5] = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    }
    for (uint8_t i = 0; i < 6; i++){
        Air_Filter->xhat->data[i][0] += 0.5*d_t*(Air_Filter_xdot[i] + Air_Filter_xdot_last2[i]);
        Air_Filter_xdot_last2[i] = Air_Filter_xdot[i];
    }

    if (Inhibit_Update){
        return;
    }
    // Update state transition matrix
    Air_Filter_State_Transition();

    // Propogate system covariances
    bool KF_Prediction_Status = Predict_EKF(Air_Filter);
    assert(KF_Prediction_Status);
}

void Air_Filter_Predict_p32(bool Inhibit_Update){
    const double d_t = 1.0/200.0;
    const double Process_Noise[3] = {0.001 * Gyro_RMS_Noise * d_t,
                                     0.001 * Gyro_RMS_Noise * d_t,
                                     0.1 * Gyro_RMS_Noise * d_t};
    Air_Filter_Covariance P_new;

    double phi = Air_Filter_xhat[3];
    double theta = Air_Filter_xhat[4];
    // If there are gyro biases, remove them from inputs
    double p, q, r;
    if (Use_Heap_Gnd){
        p = IMU_Angular_Rate(0)*D2R - Ground_Filter->xhat->data[3][0];
        q = IMU_Angular_Rate(1)*D2R - Ground_Filter->xhat->data[4][0];
        r = IMU_Angular_Rate(2)*D2R - Ground_Filter->xhat->data[5][0];
    }
    else{
        p = IMU_Angular_Rate(0)*D2R - Ground_Filter_xhat[3];
        q = IMU_Angular_Rate(1)*D2R - Ground_Filter_xhat[4];
        r = IMU_Angular_Rate(2)*D2R - Ground_Filter_xhat[5];
    }

    if (Air_Filter_xhat[2] < MIN_MU){
        Air_Filter_xhat[2] = MIN_MU;
    }
    // Perform nonlinear prediction
    Air_Filter_xdot[0] = -g_gravity*sin(theta) - (Air_Filter_xhat[2]*Air_Filter_xhat[0]/g_mass);
    Air_Filter_xdot[1] = g_gravity*sin(phi)*cos(theta) - (Air_Filter_xhat[2]*Air_Filter_xhat[1]/g_mass);
    Air_Filter_xdot[4] = cos(phi)*q - sin(phi)*r;
    if (abs(theta) != M_PI/2.0){
        Air_Filter_xdot[3] = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
        Air_Filter_xdot[5] = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    }
    // Integrate using trapezoidal rule
    for (uint8_t i = 0; i < 6; i++){
        Air_Filter_xhat[i] += 0.5*d_t*(Air_Filter_xdot[i] + Air_Filter_xdot_last[i]);
        Air_Filter_xdot_last[i] = Air_Filter_xdot[i];
    }

    if (Inhibit_Update){
        return;
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
    
    P_new.P4_4 = P_AF.P4_4 + Process_Noise[0];
    P_new.P4_5 = P_AF.P4_5;
    P_new.P4_6 = P_AF.P4_6;
    
    P_new.P5_5 = P_AF.P5_5 + Process_Noise[1];
    P_new.P5_6 = P_AF.P5_6;
    
    P_new.P6_6 = P_AF.P6_6 + Process_Noise[2];
    
    P_AF = P_new;
}

void Air_Filter_Update(){
    Matrix *measurement = Mat_Constructor(3, 1);

    // Perform measurements
    double phi = Air_Filter->xhat->data[3][0];
    double theta = Air_Filter->xhat->data[4][0];

    measurement->data[0][0] = IMU_Acceleration(0)*g_gravity;
    measurement->data[1][0] = IMU_Acceleration(1)*g_gravity;

    double mag_x_NED = cos(theta)*Magnetometer_Filtered_Field(0) + sin(phi)*sin(theta)*Magnetometer_Filtered_Field(1) + cos(phi)*sin(theta)*Magnetometer_Filtered_Field(2);
    double mag_y_NED = cos(phi)*Magnetometer_Filtered_Field(1) - sin(phi)*Magnetometer_Filtered_Field(2);
    measurement->data[2][0] = -atan2(mag_y_NED, mag_x_NED);
    if (isnan(measurement->data[2][0])){
        measurement->data[2][0] = Air_Filter->xhat->data[5][0];
    }

    // Update measurement matrix
    double u = Air_Filter->xhat->data[0][0];
    double v = Air_Filter->xhat->data[1][0];
    double mu = Air_Filter->xhat->data[2][0];

    Air_Filter->H->data[0][0] = -mu/g_mass;
    Air_Filter->H->data[0][2] = -u/g_mass;
    Air_Filter->H->data[1][1] = -mu/g_mass;
    Air_Filter->H->data[1][2] = -v/g_mass;

    // Predict what the measurement should be based on current state estimates
    Matrix *predicted_measurement = Mat_Constructor(3, 1);
    predicted_measurement->data[0][0] = -(mu*u)/g_mass;
    predicted_measurement->data[1][0] = -(mu*v)/g_mass;
    predicted_measurement->data[2][0] = Air_Filter->xhat->data[5][0];

    Matrix *measurement_error = Mat_Sub(measurement, predicted_measurement, 3);
    // This accounts for jump from -pi -> pi in atan2 function
    if (measurement_error->data[2][0] > M_PI){
        measurement_error->data[2][0] -= (2.0*M_PI);
    }
    else if (measurement_error->data[2][0] < -M_PI){
        measurement_error->data[2][0] += (2.0*M_PI);
    }

    // Update state and covariances with measurement
    Update_EKF(Air_Filter, measurement_error);

    Mat_Destructor(measurement_error);
}

void Air_Filter_Update_p32(){
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
    
    double mag_x_NED = cos(theta)*Magnetometer_Filtered_Field(0) + sin(phi)*sin(theta)*Magnetometer_Filtered_Field(1) + cos(phi)*sin(theta)*Magnetometer_Filtered_Field(2);
    double mag_y_NED = cos(phi)*Magnetometer_Filtered_Field(1) - sin(phi)*Magnetometer_Filtered_Field(2);
    
    measurement[2] = -atan2(mag_y_NED, mag_x_NED);
    if (isnan(measurement[2])){
        measurement[2] = Air_Filter_xhat[5];
    }
    
    double ybar[3] = {
        measurement[0] + ((mu*u)/g_mass),
        measurement[1] + ((mu*v)/g_mass),
        measurement[2] - Air_Filter_xhat[5]
    };
    // This accounts for jump from -pi -> pi in atan2 function
    if (ybar[2] > M_PI){
        ybar[2] -= (2.0*M_PI);
    }
    else if (ybar[2] < -M_PI){
        ybar[2] += (2.0*M_PI);
    }
    // Update measurement matrix
    double c1 = -mu/g_mass;
    double c2 = -u/g_mass;
    double c3 = -v/g_mass;
    
    // Calculate innovation
    S.data[0][0] = P_AF.P1_1*pow(c1, 2) + 2.0*P_AF.P1_3*c1*c2 + P_AF.P3_3*pow(c2, 2) + R[0];
    S.data[0][1] = c3*(P_AF.P1_3*c1 + P_AF.P3_3*c2) + c1*(P_AF.P1_2*c1 + P_AF.P2_3*c2);
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
    
    P_new.P1_1 = P_AF.P1_1*(1.0 - K[0][0]*c1) - P_AF.P1_3*(K[0][0]*c2 + K[0][1]*c3) - K[0][2]*P_AF.P1_6 - K[0][1]*P_AF.P1_2*c1;
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

void Air_Filter_State_Transition(){
    double d_t = 1.0/200.0;//Gyro_ODR;
    double u = Air_Filter->xhat->data[0][0];
    double v = Air_Filter->xhat->data[1][0];
    double mu = Air_Filter->xhat->data[2][0];
    double phi = Air_Filter->xhat->data[3][0];
    double theta = Air_Filter->xhat->data[4][0];

    Air_Filter->F->data[0][0] = 1.0 - d_t*(mu/g_mass);
    Air_Filter->F->data[0][2] = -d_t*(u/g_mass);
    Air_Filter->F->data[0][4] = -d_t*g_gravity*cos(theta);

    Air_Filter->F->data[1][1] = 1.0- d_t*(mu/g_mass);
    Air_Filter->F->data[1][2] = -d_t*(v/g_mass);
    Air_Filter->F->data[1][3] = d_t*g_gravity*cos(theta)*cos(phi);
    Air_Filter->F->data[1][4] = -d_t*g_gravity*sin(theta)*sin(phi);
}

double Air_Filter_data(uint8_t index){
    if (Use_Heap_Air || Use_Heap_All){
        return Air_Filter->xhat->data[index][0];
    }
    else{
        return Air_Filter_xhat[index];
    }
}

double Air_Filter_x_Dot(uint8_t index){
    return Air_Filter_xdot[index];
}

void Altitude_Filter_Predict(){
    Matrix *Accel_Input = Mat_Constructor(1, 1);

    double phi = Air_Filter_data(3);
    double theta = Air_Filter_data(4);
    double Accel_Up = -(-sin(theta)*IMU_Acceleration(0) + sin(phi)*cos(theta)*IMU_Acceleration(1) + cos(phi)*cos(theta)*IMU_Acceleration(2) + 1.0)*g_gravity;
    Accel_Input->data[0][0] = Accel_Up;

    Predict(Altitude_Filter, Accel_Input);

    Mat_Destructor(Accel_Input);
}

void Altitude_Filter_Update(){
    Matrix *Bar_Measurement = Mat_Constructor(1, 1);

    Bar_Measurement->data[0][0] = Barometer_Altitude();
    
    bool status = Update(Altitude_Filter, Bar_Measurement);
    assert(status);
    Mat_Destructor(Bar_Measurement);
}

void Altitude_Filter_Predict_p32(){
    const double d_t = 1.0/200.0;
    double Altitude_Filter_xdot[2];
    double phi = Air_Filter_data(3);
    double theta = Air_Filter_data(4);
    double Accel_Up = -(-sin(theta)*IMU_Acceleration(0) + sin(phi)*cos(theta)*IMU_Acceleration(1) + cos(phi)*cos(theta)*IMU_Acceleration(2) + 1.0)*g_gravity;

    Altitude_Filter_xdot[1] = Accel_Up;
    Altitude_Filter_xdot[0] = Altitude_Filter_xhat[1];

    for (uint8_t i = 0; i < 2; i++){
        Altitude_Filter_xhat[i] += 0.5*d_t*(Altitude_Filter_xdot[i] + Altitude_Filter_xdot_last[i]);
        Altitude_Filter_xdot_last[i] = Altitude_Filter_xdot[i];
    }
}

void Altitude_Filter_Update_p32(){
    // Kalman gain 
    const double K[2] = {0.011971, 0.005376};

    double ybar = Barometer_Altitude() - Altitude_Filter_xhat[0];
    Altitude_Filter_xhat[0] += K[0]*ybar;
    Altitude_Filter_xhat[1] += K[1]*ybar;
}

double Altitude_Filter_data(uint8_t index){
    if (Use_Heap_Alt || Use_Heap_All){
        return Altitude_Filter->xhat->data[index][0];
    }
    else{
        return Altitude_Filter_xhat[index];
    }
}