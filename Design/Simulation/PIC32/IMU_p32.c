#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "IMU_p32.h"
#include "Magnetometer_p32.h"
#include "Barometer_p32.h"
#include "Sim_Types.h"
#include "Global_Variables_p32.h"

// Flag to control whether to use the stack implementation of the kalman filters or the heap implementation
static const bool Use_Heap = false;

// Stores variables for use by other systems
static IMU_Data IMU_variables;
// QuadrotorsandAccelerometers.pdf
static Kalman_Filter *Ground_Filter;
static Kalman_Filter *Air_Filter;
static Kalman_Filter *Altitude_Filter;
static Matrix *input;
static Matrix *measurement;
// In units of Hz
static double Gyro_ODR;
// Date sheet gives value of 5mdps/sqrt(ODR), convert to rad/s
static double Gyro_RMS_Noise;
// In units of Hz
static double Accel_ODR;
// Data sheet gives value of 90ug/sqrt(ODR), convert to g
static double Accel_RMS_Noise;
// Assume drift rate of 20 deg/hr, convert to rad/s
const double Gyro_Bias_Instability = (20.0/3600.0)*D2R;
// Constant value if LPF = ODR/2, normalize by dividing by average field strength
const double Mag_RMS_Noise = 4.5/500.0;
// Given the ODR is 75Hz and the LPF setting is ODR/20, 0.65 in Pa which at STP corresponds to 0.054 m
const double BAR_RMS_Noise = 0.054;

static double Air_Filter_xdot_last[6];
static double Air_Filter_xdot[6];
static double Altitude_Filter_xdot_last[2];
static double Altitude_Filter_xdot[2];
// In units of Hz
static const double Gyro_ODR_2 = 57.706152185014034;//40.743097574926725;
static const double Gyro_RMS_Noise_p32 = 0.005*Gyro_ODR_2*D2R;
static const double d_t_p32 = 1.0/3330.0;
// In units of Hz
static double Air_Filter_xhat[6];
static double Altitude_Filter_xhat[2];
static Air_Filter_Covariance P_AF;
static const Air_Filter_Covariance A_Process_Cov = {
    .P4_4 = 0.01*Gyro_RMS_Noise_p32*d_t_p32,
    .P5_5 = 0.01*Gyro_RMS_Noise_p32*d_t_p32,
    .P6_6 = 0.01*Gyro_RMS_Noise_p32*d_t_p32
};

void Set_ODR(double gyro_ODR, double accel_ODR){
    Gyro_ODR = gyro_ODR;
    Gyro_RMS_Noise = 0.005*sqrt(Gyro_ODR)*D2R;
    Accel_ODR = accel_ODR;
    Accel_RMS_Noise = 0.00009*sqrt(Accel_ODR);
}

void Read_Gyro(Time Current_Time, int16_t angular_rate_LSB[3]){
    const double gyro_sens = 500.0/32768.0;
	
	IMU_variables.gyro_output_LSB[0] = angular_rate_LSB[0];
	IMU_variables.gyro_output_LSB[1] = -angular_rate_LSB[1];
	IMU_variables.gyro_output_LSB[2] = -angular_rate_LSB[2];

    IMU_variables.angular_rate[0] = ((double)IMU_variables.gyro_output_LSB[0])*gyro_sens;
    IMU_variables.angular_rate[1] = ((double)IMU_variables.gyro_output_LSB[1])*gyro_sens;
    IMU_variables.angular_rate[2] = ((double)IMU_variables.gyro_output_LSB[2])*gyro_sens;
}

void Read_Accel(int16_t acceleration_LSB[3]){
    const double accel_sens = 2.0/32768.0;

	IMU_variables.accel_output_LSB[0] = acceleration_LSB[0];
	IMU_variables.accel_output_LSB[1] = -acceleration_LSB[1];
	IMU_variables.accel_output_LSB[2] = -acceleration_LSB[2];

    IMU_variables.acceleration[0] = ((double)IMU_variables.accel_output_LSB[0])*accel_sens;
    IMU_variables.acceleration[1] = ((double)IMU_variables.accel_output_LSB[1])*accel_sens;
    IMU_variables.acceleration[2] = ((double)IMU_variables.accel_output_LSB[2])*accel_sens;

    for (uint8_t i = 0; i < 3; i++){
		bool calculate_offset = false;
		if (IMU_variables.accel_output_LSB[i] > IMU_variables.accel_max[i]){
			IMU_variables.accel_max[i] = IMU_variables.accel_output_LSB[i];
			calculate_offset = true;
		}
		else if (IMU_variables.accel_output_LSB[i] < IMU_variables.accel_min[i]){
			IMU_variables.accel_min[i] = IMU_variables.accel_output_LSB[i];
			calculate_offset = true;
		}
		if (calculate_offset){
            IMU_variables.accel_bias_LSB[i] = (int32_t)IMU_variables.accel_max[i] + (int32_t)IMU_variables.accel_min[i];
			IMU_variables.accel_bias_LSB[i] >>= 1;
		}
	}
}

bool Initialize_IMU_Filters(){
    memset(&IMU_variables, 0, sizeof(IMU_variables));
    memset(Air_Filter_xhat, 0, sizeof(Air_Filter_xhat));
    memset(&P_AF, 0, sizeof(P_AF));
    memset(Air_Filter_xdot_last, 0, sizeof(Air_Filter_xdot_last));
    memset(Air_Filter_xdot, 0, sizeof(Air_Filter_xdot));
    memset(Altitude_Filter_xhat, 0, sizeof(Altitude_Filter_xhat));
    memset(Altitude_Filter_xdot_last, 0, sizeof(Altitude_Filter_xdot_last));
    memset(Altitude_Filter_xdot, 0, sizeof(Altitude_Filter_xdot));

    // The ground kalman filter will have 9 states, 3 inputs, and 3 measurements
    if (Ground_Filter != NULL) Filter_Destructor(Ground_Filter, 6);
    Ground_Filter = Filter_Constructor(9, 3, 3);
    if (Ground_Filter == NULL) return false;

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
    double d_t = 1.0/Gyro_ODR;

    Ground_Filter->Q->data[0][0] = Gyro_RMS_Noise;
    Ground_Filter->Q->data[1][1] = Gyro_RMS_Noise;
    Ground_Filter->Q->data[2][2] = Gyro_RMS_Noise;
    Ground_Filter->Q->data[3][3] = Gyro_Bias_Instability;
    Ground_Filter->Q->data[4][4] = Gyro_Bias_Instability;
    Ground_Filter->Q->data[5][5] = Gyro_Bias_Instability;
    Ground_Filter->Q->data[6][6] = Gyro_RMS_Noise*d_t;
    Ground_Filter->Q->data[7][7] = Gyro_RMS_Noise*d_t;
    Ground_Filter->Q->data[8][8] = Gyro_RMS_Noise*d_t;

    Ground_Filter->R->data[0][0] = Accel_RMS_Noise;
    Ground_Filter->R->data[1][1] = Accel_RMS_Noise;
    Ground_Filter->R->data[2][2] = Mag_RMS_Noise;
    
    if (input != NULL) Mat_Destructor(input);
    input = Mat_Constructor(3, 1);
    if (input == NULL) return false;

    if (measurement != NULL) Mat_Destructor(measurement);
    measurement = Mat_Constructor(3, 1);
    if (measurement == NULL) return false;

    // The air kalman filter will have 6 states, 3 inputs, 3 measurements
    if (Air_Filter != NULL) Filter_Destructor(Air_Filter, 6);
    Air_Filter = Filter_Constructor(6, 3, 3);
    if (Air_Filter == NULL) return false;

    Air_Filter->F->data[2][2] = 1.0;
    Air_Filter->F->data[3][3] = 1.0;
    Air_Filter->F->data[4][4] = 1.0;
    Air_Filter->F->data[5][5] = 1.0;

    Air_Filter->H->data[2][5] = 1.0;
    
    // Initialize process, state, and system covariances
    Air_Filter->P->data[2][2] = 0.5;

    Air_Filter->Q->data[3][3] = 0.01*Gyro_RMS_Noise*d_t;
    Air_Filter->Q->data[4][4] = 0.01*Gyro_RMS_Noise*d_t;
    Air_Filter->Q->data[5][5] = 0.01*Gyro_RMS_Noise*d_t;

    Air_Filter->R->data[0][0] = Accel_RMS_Noise*g_gravity;
    Air_Filter->R->data[1][1] = Accel_RMS_Noise*g_gravity;
    Air_Filter->R->data[2][2] = Mag_RMS_Noise;

    if (Altitude_Filter != NULL) Filter_Destructor(Altitude_Filter, 6);
    Altitude_Filter = Filter_Constructor(2, 1, 1);
    if (Altitude_Filter == NULL) return false;

    Altitude_Filter->F->data[0][0] = 1.0;
    Altitude_Filter->F->data[0][1] = 1.0/Accel_ODR;
    Altitude_Filter->F->data[1][1] = 1.0;

    Altitude_Filter->B->data[1][0] = 1.0/Accel_ODR;

    Altitude_Filter->H->data[0][0] = 1.0;

    Altitude_Filter->Q->data[1][1] = Accel_RMS_Noise*g_gravity*(1.0/Accel_ODR);

    Altitude_Filter->R->data[0][0] = BAR_RMS_Noise;

    return true;
}

// Before take off, assume that normal forces acting on the drone enable measuring
// of the inverse g_gravity vector through these normal forces
// An initial kalman filter will be run under these assumptions in order to estimate
// initial attitude conditions and gyro bias instability
void Ground_Filter_Predict(){
    // States are : w_x, w_y, w_z, w_x_bias, w_y_bias, w_z_bias, phi, theta, psi
    // Inputs are : p, q, r
    // Measurements are: phi, theta, psi

    // Update state transition matrix based on updated states
    Ground_Filter_State_Transition();

    // Perform prediction
    input->data[0][0] = IMU_variables.angular_rate[0]*D2R;
    input->data[1][0] = IMU_variables.angular_rate[1]*D2R;
    input->data[2][0] = IMU_variables.angular_rate[2]*D2R;

    bool KF_Prediction_Status = Predict(Ground_Filter, input);
    assert(KF_Prediction_Status);
}

void Ground_Filter_Update(double mag_field[3]){
    // Perform roll-pitch-yaw measurements
    double phi = Ground_Filter->xhat->data[6][0];
    double theta = Ground_Filter->xhat->data[7][0];
    measurement->data[0][0] = atan2((-IMU_variables.acceleration[1]), (-IMU_variables.acceleration[2]));
    if (isnan( measurement->data[0][0])){
        measurement->data[0][0] = phi;
    }
    measurement->data[1][0] = atan2((IMU_variables.acceleration[0]), sqrt(pow(IMU_variables.acceleration[1],2) + pow(IMU_variables.acceleration[2],2)));
    if (isnan(measurement->data[1][0])){
        measurement->data[1][0] = theta;
    }

    double mag_x_NED = cos(theta)*mag_field[0] + sin(phi)*sin(theta)*mag_field[1] + cos(phi)*sin(theta)*mag_field[2];
    double mag_y_NED = cos(phi)*mag_field[1] - sin(phi)*mag_field[2];
    measurement->data[2][0] = -atan2(mag_y_NED, mag_x_NED);
    if (isnan(measurement->data[2][0])){
        measurement->data[2][0] = Ground_Filter->xhat->data[8][0];
    }

    // Update state and covariances with measurement
    Update(Ground_Filter, measurement);
}

void Ground_Filter_State_Transition(){
    double d_t = 1.0/Accel_ODR;
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

double Filter_data(uint8_t index, uint8_t filter){
    if (Use_Heap){
        switch(filter){
            case 0:
                return Ground_Filter->xhat->data[index][0];
            case 1:
                return Air_Filter->xhat->data[index][0];
            case 2:
                return Altitude_Filter->xhat->data[index][0];
        }
    }
    else{
        switch(filter){
            case 0:
                return Ground_Filter->xhat->data[index][0];
            case 1:
                return Air_Filter_xhat[index];
            case 2:
                return Altitude_Filter_xhat[index];
        }
    }

    return 0.0;
}

double Filter_covariance(uint8_t index1, uint8_t index2, uint8_t filter){
    switch(filter){
        case 0:
            return Ground_Filter->P->data[index1][index2];
        case 1:
            return Air_Filter->P->data[index1][index2];
        case 2:
            return Altitude_Filter->P->data[index1][index2];
    }

    return 0.0;
}

// After take off, assume that rotor drag force in the body x and y directions
// is proportional to the product of the body translational rate and some drag constant mu
// the accelerometers on the body x and y axes will measure this force and can be used to estimate 
// mu, roll, and pitch
void Air_Filter_Predict(){
    // States are : u, v, mu, phi, theta, psi
    // Inputs are : p, q, r
    // Measurements are: (mu/m)*u, (mu/m)*v, psi
    double d_t = 1.0/Gyro_ODR;
    double phi = Air_Filter->xhat->data[3][0];
    double theta = Air_Filter->xhat->data[4][0];
    double p = IMU_variables.angular_rate[0]*D2R - Ground_Filter->xhat->data[3][0];
    double q = IMU_variables.angular_rate[1]*D2R - Ground_Filter->xhat->data[4][0];
    double r = IMU_variables.angular_rate[2]*D2R - Ground_Filter->xhat->data[5][0];
    // Perform nonlinear prediction
    Air_Filter_xdot[0] = -g_gravity*sin(theta) - (Air_Filter->xhat->data[2][0]*Air_Filter->xhat->data[0][0]/g_mass);
    Air_Filter_xdot[1] = g_gravity*sin(phi)*cos(theta) - (Air_Filter->xhat->data[2][0]*Air_Filter->xhat->data[1][0]/g_mass);
    Air_Filter_xdot[4] = cos(phi)*q - sin(phi)*r;
    if (abs(theta) != M_PI/2.0){
        Air_Filter_xdot[3] = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
        Air_Filter_xdot[5] = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    }
    for (uint8_t i = 0; i < 6; i++){
        Air_Filter->xhat->data[i][0] += 0.5*d_t*(Air_Filter_xdot[i] + Air_Filter_xdot_last[i]);
        Air_Filter_xdot_last[i] = Air_Filter_xdot[i];
    }

    // Update state transition matrix
    Air_Filter_State_Transition();

    // Propogate system covariances
    bool KF_Prediction_Status = Predict_EKF(Air_Filter);
    assert(KF_Prediction_Status);
}

void Air_Filter_Predict_p32(){
    Air_Filter_Covariance P_new;

    double phi = Air_Filter_xhat[3];
    double theta = Air_Filter_xhat[4];
    // If there are gyro biases, remove them from inputs
    double p = IMU_Angular_Rate(0)*D2R - Ground_Filter->xhat->data[3][0];
    double q = IMU_Angular_Rate(1)*D2R - Ground_Filter->xhat->data[4][0];
    double r = IMU_Angular_Rate(2)*D2R - Ground_Filter->xhat->data[5][0];

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
        Air_Filter_xhat[i] += 0.5*d_t_p32*(Air_Filter_xdot[i] + Air_Filter_xdot_last[i]);
        Air_Filter_xdot_last[i] = Air_Filter_xdot[i];
    }
    
    // Update state transition matrix 
    double u = Air_Filter_xhat[0];
    double v = Air_Filter_xhat[1];
    double mu = Air_Filter_xhat[2];
    phi = Air_Filter_xhat[3];
    theta = Air_Filter_xhat[4];

    double c1 = 1.0 - d_t_p32*(mu/g_mass);
    double c2 = -d_t_p32*(u/g_mass);
    double c3 = -d_t_p32*g_gravity*cos(theta);
    double c4 = -d_t_p32*(v/g_mass);
    double c5 = d_t_p32*g_gravity*cos(theta)*cos(phi);
    double c6 = -d_t_p32*g_gravity*sin(theta)*sin(phi);
    
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
    P_new.P4_5 = P_AF.P4_5;
    P_new.P4_6 = P_AF.P4_6;
    
    P_new.P5_5 = P_AF.P5_5 + A_Process_Cov.P5_5;
    P_new.P5_6 = P_AF.P5_6;
    
    P_new.P6_6 = P_AF.P6_6 + A_Process_Cov.P6_6;
    
    P_AF = P_new;
}

void Air_Filter_Update(){
    // Perform measurements
    double phi = Air_Filter->xhat->data[3][0];
    double theta = Air_Filter->xhat->data[4][0];

    measurement->data[0][0] = IMU_variables.acceleration[0]*g_gravity;
    measurement->data[1][0] = IMU_variables.acceleration[1]*g_gravity;

    double mag_x_NED = cos(theta)*Magnetometer_Field(0) + sin(phi)*sin(theta)*Magnetometer_Field(1) + cos(phi)*sin(theta)*Magnetometer_Field(2);
    double mag_y_NED = cos(phi)*Magnetometer_Field(1) - sin(phi)*Magnetometer_Field(2);
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

    // Update state and covariances with measurement
    Update_EKF(Air_Filter, measurement, predicted_measurement);
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
    double d_t = 1.0/Gyro_ODR;
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

double Air_Filter_x_Dot(uint8_t index){
    return Air_Filter_xdot[index];
}

double IMU_Acceleration(uint8_t index){
    return IMU_variables.acceleration[index];
}

double IMU_Angular_Rate(uint8_t index){
    return IMU_variables.angular_rate[index];
}

void Initialize_Air_Filter(){
    if (Use_Heap){
        Air_Filter->xhat->data[2][0] = 0.5;
        Air_Filter->xhat->data[3][0] = Ground_Filter->xhat->data[6][0];
        Air_Filter->xhat->data[4][0] = Ground_Filter->xhat->data[7][0];
        Air_Filter->xhat->data[5][0] = Ground_Filter->xhat->data[8][0];
    }
    else{
        // Seed drag coefficient uncertainties so that they will converge faster
        P_AF.P3_3 = 0.5;
        // The initial conditions for the Euler angle estimates will be taken from the ground filter states
        Air_Filter_xhat[3] = Ground_Filter->xhat->data[6][0];
        Air_Filter_xhat[4] = Ground_Filter->xhat->data[7][0];
        Air_Filter_xhat[5] = Ground_Filter->xhat->data[8][0];
        // Make an initial guess of the drag coefficient
        Air_Filter_xhat[2] = 0.5;
    }

}

void Altitude_Filter_Predict(){
    Matrix *Accel_Input = Mat_Constructor(1, 1);

    double phi = Filter_data(3, 1);
    double theta = Filter_data(4, 1);
    double Accel_Down = -(-sin(theta)*IMU_Acceleration(0) + sin(phi)*cos(theta)*IMU_Acceleration(1) + cos(phi)*cos(theta)*IMU_Acceleration(2) + 1.0)*g_gravity;
    Accel_Input->data[0][0] = Accel_Down;

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
    const double d_t = 1.0/1660.0;
    double phi = Filter_data(3, 1);
    double theta = Filter_data(4, 1);
    double Accel_Down = -(-sin(theta)*IMU_Acceleration(0) + sin(phi)*cos(theta)*IMU_Acceleration(1) + cos(phi)*cos(theta)*IMU_Acceleration(2) + 1.0)*g_gravity;

    Altitude_Filter_xdot[1] = Accel_Down;
    Altitude_Filter_xdot[0] = Altitude_Filter_xhat[1] + 0.5*d_t*Accel_Down;

    for (uint8_t i = 0; i < 2; i++){
        Altitude_Filter_xhat[i] += 0.5*d_t*(Altitude_Filter_xdot[i] + Altitude_Filter_xdot_last[i]);
        Altitude_Filter_xdot_last[i] = Altitude_Filter_xdot[i];
    }
}

void Altitude_Filter_Update_p32(){
    // Kalman gain 
    const double K[2] = {0.0212, 0.0529};

    double ybar = Barometer_Altitude() - Altitude_Filter_xhat[0];
    Altitude_Filter_xhat[0] += K[0]*ybar;
    Altitude_Filter_xhat[1] += K[1]*ybar;
}

// CPP wrapper functions
void Set_ODR_cpp(double gyro_ODR, double accel_ODR){
    Set_ODR(gyro_ODR, accel_ODR);
}

double IMU_Angular_Rate_cpp(uint8_t index){
    return IMU_Angular_Rate(index);
}

double IMU_Acceleration_cpp(uint8_t index){
    return IMU_Acceleration(index);
}

double Filter_data_cpp(uint8_t index, uint8_t filter){
    return Filter_data(index, filter);
}

double Filter_covariance_cpp(uint8_t index1, uint8_t index2, uint8_t filter){
    return Filter_covariance(index1, index2, filter);
}

void Read_Gyro_cpp(Time Current_Time, int16_t angular_rate_LSB[3]){
    Read_Gyro(Current_Time, angular_rate_LSB);
}

bool Initialize_IMU_Filters_cpp(){
    return Initialize_IMU_Filters();
}

void Ground_Filter_Update_cpp(double mag_field[3]){
    Ground_Filter_Update(mag_field);
}

void Ground_Filter_Predict_cpp(){
    Ground_Filter_Predict();
}

void Air_Filter_Predict_cpp(){
    if (Use_Heap){
        Air_Filter_Predict();
    }
    else{
        Air_Filter_Predict_p32();
    }
}

void Air_Filter_Update_cpp(){
    if (Use_Heap){
        Air_Filter_Update();
    }
    else{ 
        Air_Filter_Update_p32();
    }
}

void Initialize_Air_Filter_cpp(){
    Initialize_Air_Filter();
}

void Read_Accel_cpp(int16_t acceleration_LSB[3]){
    Read_Accel(acceleration_LSB);
}

void Altitude_Filter_Predict_cpp(){
    if (Use_Heap){
        Altitude_Filter_Predict();
    }
    else {
        Altitude_Filter_Predict_p32();
    }
}

void Altitude_Filter_Update_cpp(){
    if (Use_Heap){
        Altitude_Filter_Update();
    }
    else{
        Altitude_Filter_Update_p32();
    }
}