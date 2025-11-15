#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "Magnetometer_p32.h"
#include "Global_Variables_p32.h"
#include "External_Interface.h"
#include "Butterworth_Filter_p32.h"

static Mag_Machine state;
static Mag_Data mag = {0};
static BW_Filter_Data BW_Filter[3];
static Time Last_Filter;
const int32_t Filter_Rate_Hz = 3000;

void Initialize_Mag(){
    memset(&mag, 0, sizeof(mag));
    memset(&Last_Filter, 0, sizeof(Last_Filter));
    Initialize_BW_Filter(&BW_Filter[0], 1.0/((double)Filter_Rate_Hz));
    Initialize_BW_Filter(&BW_Filter[1], 1.0/((double)Filter_Rate_Hz));
    Initialize_BW_Filter(&BW_Filter[2], 1.0/((double)Filter_Rate_Hz));
    BW_Filter[0].w_c = 2.0;
    BW_Filter[1].w_c = 2.0;
    BW_Filter[2].w_c = 2.0;
    mag.mag_field_min_LSB[0] = INT16_MAX;
    mag.mag_field_min_LSB[1] = INT16_MAX;
    mag.mag_field_min_LSB[2] = INT16_MAX;
    mag.mag_field_max_LSB[0] = INT16_MIN;
    mag.mag_field_max_LSB[1] = INT16_MIN;
    mag.mag_field_max_LSB[2] = INT16_MIN;
    mag.offset_initialized[0] = false;
    mag.offset_initialized[1] = false;
    mag.offset_initialized[2] = false;
    state = Mag_Standby;
}

void Run_Magnetometer_Machine(){
    const int32_t ODR_Hz = 100;
    const Time Sample_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Hz};
    const Time Filter_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/Filter_Rate_Hz};
    bool Hard_iron_cal = false;
    bool Soft_iron_cal = false;
    
    switch (state){
       case Mag_Standby:
           e_mag_odr = ODR_Hz;
           e_mag_lpf_setting = 1;
           e_mag_settings_updated = true;
           state = Mag_Ready;
           break;
       case Mag_Fail:
           break;
       case Mag_Ready:
           if (Compare_And_Update(Current_Time(), Sample_Rate, &mag.Last_Update)){
               state = Mag_Reading;
           }
            if (g_Flight_Controller_Status == System_Calibration){
                if (Compare_And_Update(Current_Time(), Filter_Rate, &Last_Filter)){
                    for (uint8_t i = 0; i < 3; i++){
                        BW_Filter[i].u = mag.field[i];
                        mag.field_filtered[i] = Run_BW_Filter(&BW_Filter[i]);
                    }
                }
            }
            else if (g_Flight_Controller_Status > System_Calibration){
                BW_Filter[0].w_c = 30.0;
                BW_Filter[1].w_c = 30.0;
                BW_Filter[2].w_c = 30.0;
                if (Compare_And_Update(Current_Time(), Filter_Rate, &Last_Filter)){
                    for (uint8_t i = 0; i < 3; i++){
                        BW_Filter[i].u = mag.field[i];
                        mag.field_filtered[i] = Run_BW_Filter(&BW_Filter[i]);
                    }
                }
            }
            else{
                memcpy(&mag.field_filtered, &mag.field, sizeof(mag.field_filtered));
            }
           break;
       case Mag_Reading:
            Convert_Magnetometer();
            // If the current measurements require the hard iron offsets to be updated, 
            // then a new soft iron calibration will also be required
            Hard_iron_cal = Calculate_Hard_Iron();
            if (Hard_iron_cal){
                Soft_iron_cal = Calculate_Soft_Iron();
            }
            Compensate_Magnetometer_Reading(Soft_iron_cal);

            // if(g_Flight_Controller_Status < Flying){
            //     Magnetometer_LPF(0);
            // }
            // else {
            //     Magnetometer_LPF(1);
            // }

           state = Mag_Ready;
           break;
    }
}

void Convert_Magnetometer(){
    // The Magnetometer sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor y
    // -> Body y = -Sensor x
    // -> Body z = Sensor z

    memcpy(mag.field_LSB_bytes, e_mag_data, sizeof(e_mag_data));

    mag.field_LSB[0] = (((int16_t)mag.field_LSB_bytes[3])<<8) + ((int16_t)mag.field_LSB_bytes[2]);
    mag.field_LSB[1] = -(((int16_t)mag.field_LSB_bytes[1])<<8) - ((int16_t)mag.field_LSB_bytes[0]);
    mag.field_LSB[2] = (((int16_t)mag.field_LSB_bytes[5])<<8) + ((int16_t)mag.field_LSB_bytes[4]);

}

bool Calculate_Hard_Iron(){
    // We will find the ellipsoid center offset from 0 by saving off the
    // maximum and minimum measured values and dividing by two
    bool update_performed = false;
    
	for (uint8_t i = 0; i < 3; i++){
		bool calculate_hard_iron = false;
		if (mag.field_LSB[i] > mag.mag_field_max_LSB[i]){
			mag.mag_field_max_LSB[i] = mag.field_LSB[i];
            mag.mag_values_at_max[i][0] = mag.field_LSB[0];
            mag.mag_values_at_max[i][1] = mag.field_LSB[1];
            mag.mag_values_at_max[i][2] = mag.field_LSB[2];
			calculate_hard_iron = true;
		}
		if (mag.field_LSB[i] < mag.mag_field_min_LSB[i]){
			mag.mag_field_min_LSB[i] = mag.field_LSB[i];
			calculate_hard_iron = true;
		}
		if (calculate_hard_iron){
			mag.hard_iron[i] = mag.mag_field_min_LSB[i] + mag.mag_field_max_LSB[i];
			mag.hard_iron[i] >>= 1;
            mag.offset_initialized[i] = (mag.mag_field_max_LSB[i] != mag.hard_iron[i])?true:false;
            update_performed = true;
		}
	}
    
	// Prevents future divide by 0 error in soft iron offset calculation
    return (update_performed && (mag.offset_initialized[0] & mag.offset_initialized[1] & mag.offset_initialized[2]))?true:false;

}

bool Calculate_Soft_Iron(){
    // Soft iron calibration is performed by finding the linear transformation
    // that turns a sphere into the measured ellipsoid with semi axes defined as
    // the maximum values in each sensor direction after hard iron offsets are removed.
    // The eigenvalues (lambda) of the transformation correspond to the semi axes (s) in the following way:
    // s_i = 1/sqrt(lambda_i) -> lambda_i = 1/s_i^2
    // And the eigenvectors will be the axes directions
    // Reference : https://see.stanford.edu/materials/lsoeldsee263/15-symm.pdf slide 18
    
    double lambda[3];
    double largest_lambda = 0.0;
    double Col_magnitude;
    int16_t semi_axis[3];
    Matrix_3 G = {0};
    Matrix_3 Q = {0}; 
    Matrix_3 Q_inv = {0};
    Matrix_3 Temp1 = {0};
    
    // The transformation matrix A is described as A = Q*G*Q^-1, where Q is a matrix whose columns are the eigenvectors of A
    // And G is a matrix with the eigenvalues of A are on the diagonal

    for (uint8_t i = 0; i < 3; i++){
        semi_axis[i] = mag.mag_field_max_LSB[i] - mag.hard_iron[i];
        lambda[i] = 1.0/pow((double)semi_axis[i], 2);
        if (lambda[i] > largest_lambda){
            largest_lambda = lambda[i];
        }
        G.data[i][i] = lambda[i];
    }
    
    // Build Q
    for (uint8_t i = 0; i < 3; i++){
        Q.data[0][i] = (double)(mag.mag_values_at_max[0][i] - mag.hard_iron[i]);
        Q.data[1][i] = (double)(mag.mag_values_at_max[1][i] - mag.hard_iron[i]);
        Q.data[2][i] = (double)(mag.mag_values_at_max[2][i] - mag.hard_iron[i]);
    }
    
    // Normalize eigenvectors and eigenvalues
    for (uint8_t i = 0; i < 3; i++){
        Col_magnitude = sqrt(pow(Q.data[0][i], 2) + pow(Q.data[1][i], 2) + pow(Q.data[2][i], 2));
        for (uint8_t j = 0; j < 3; j++){
            Q.data[j][i] /= Col_magnitude;
        }
        G.data[i][i] /= largest_lambda;
    }
        
    bool inv_successful = Mat3_Inv(&Q, &Q_inv);
    if (!inv_successful) return false;
    Mat3_Mul(&Q, &G, &Temp1);
    Mat3_Mul(&Temp1, &Q_inv, &mag.soft_iron);

    Mat3_Tran(&Q, &mag.soft_iron);

    return true;
}

void Compensate_Magnetometer_Reading(bool Soft_iron_cal){
    // Hard iron compensation values were calculated using magnetometer readings in LSB
    double comp_hard_iron[3];
    
    comp_hard_iron[0] = (double)(mag.field_LSB[0] - mag.hard_iron[0]);
    comp_hard_iron[1] = (double)(mag.field_LSB[1] - mag.hard_iron[1]);
    comp_hard_iron[2] = (double)(mag.field_LSB[2] - mag.hard_iron[2]);
    
    if (Soft_iron_cal){
        mag.field[0] = (mag.soft_iron.data[0][0]*comp_hard_iron[0] + mag.soft_iron.data[0][1]*comp_hard_iron[1] + mag.soft_iron.data[0][2]*comp_hard_iron[2])*MAG_SENSITIVITY;
        mag.field[1] = (mag.soft_iron.data[1][0]*comp_hard_iron[0] + mag.soft_iron.data[1][1]*comp_hard_iron[1] + mag.soft_iron.data[1][2]*comp_hard_iron[2])*MAG_SENSITIVITY;
        mag.field[2] = (mag.soft_iron.data[2][0]*comp_hard_iron[0] + mag.soft_iron.data[2][1]*comp_hard_iron[1] + mag.soft_iron.data[2][2]*comp_hard_iron[2])*MAG_SENSITIVITY;
    }
    else{
        mag.field[0] = comp_hard_iron[0]*MAG_SENSITIVITY;
        mag.field[1] = comp_hard_iron[1]*MAG_SENSITIVITY;
        mag.field[2] = comp_hard_iron[2]*MAG_SENSITIVITY;
    }

}

void Magnetometer_LPF(uint8_t setting){
    double c1;
    double c2;
    
    if (setting == 0){
        c1 = 0.995;
        c2 = 1.0 - c1;
        mag.field_filtered[0] = mag.field_filtered[0]*c1 + mag.field[0]*c2;
        mag.field_filtered[1] = mag.field_filtered[1]*c1 + mag.field[1]*c2;
        mag.field_filtered[2] = mag.field_filtered[2]*c1 + mag.field[2]*c2;
    }
    else if (setting == 1){
        c1 = 0.75;
        c2 = 1.0 - c1;
        mag.field_filtered[0] = mag.field_filtered[0]*c1 + mag.field[0]*c2;
        mag.field_filtered[1] = mag.field_filtered[1]*c1 + mag.field[1]*c2;
        mag.field_filtered[2] = mag.field_filtered[2]*c1 + mag.field[2]*c2;
    }
}

double Magnetometer_Field(uint8_t index){
    return mag.field[index];
}

double Magnetometer_Filtered_Field(uint8_t index){
    return mag.field_filtered[index];
}