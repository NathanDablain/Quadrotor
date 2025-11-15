#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "IMU_p32.h"
#include "Time_p32.h"
#include "Global_Variables_p32.h"
#include "External_Interface.h"

// Stores variables for use by other systems
static IMU_Data imu;
static IMU_Machine state_accel;
static IMU_Machine state_gyro;

void Initialize_IMU(){
    memset(&imu, 0, sizeof(imu));
    state_accel = IMU_Standby;
    state_gyro = state_accel;
}

void Run_IMU_Machine(){
    const int32_t ODR_Accel_Hz = 400;//1660;
    const int32_t ODR_Gyro_Hz = 400;//3330;
    const Time Sample_Rate_Accel = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Accel_Hz};
    const Time Sample_Rate_Gyro = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Gyro_Hz};

    // Accelerometer state machine
    switch (state_accel){
       case IMU_Standby:
		   e_accel_lpf_setting = 7;
		   e_accel_odr = 400;//1660;
		   e_gyro_lpf_setting = 5;
		   e_gyro_odr = 400;//3330;
		   e_imu_settings_updated = true;
           state_accel = IMU_Ready;
           break;
       case IMU_Fail:
           break;
       case IMU_Ready:
           if (Compare_And_Update(Current_Time(), Sample_Rate_Accel, &imu.Last_Update_Accel)){
               state_accel = IMU_Reading;
           }
           break;
       case IMU_Reading:
			Convert_Accel();
			state_accel = IMU_Ready;
           break;
   }
     
    // Gyroscope state machine
    switch (state_gyro){
       case IMU_Standby:
           state_gyro = state_accel;
           break;
       case IMU_Fail:
           break;
       case IMU_Ready:
           if (Compare_And_Update(Current_Time(), Sample_Rate_Gyro, &imu.Last_Update_Gyro)){
               state_gyro = IMU_Reading;
           }
           break;
       case IMU_Reading:
			Convert_Gyro();
			state_gyro = IMU_Ready;
           break;
   }
}

void Convert_Gyro(){
    const double gyro_sens = 500.0/32768.0;
	
	memcpy(imu.gyro_LSB_bytes, e_gyro_data, sizeof(e_gyro_data));

    imu.gyro_LSB[0] = (((int16_t)imu.gyro_LSB_bytes[1])<<8) + ((int16_t)imu.gyro_LSB_bytes[0]);
    imu.gyro_LSB[1] = -(((int16_t)imu.gyro_LSB_bytes[3])<<8) - ((int16_t)imu.gyro_LSB_bytes[2]);
    imu.gyro_LSB[2] = -(((int16_t)imu.gyro_LSB_bytes[5])<<8) - ((int16_t)imu.gyro_LSB_bytes[4]);

    imu.angular_rate[0] = ((double)imu.gyro_LSB[0])*gyro_sens;
    imu.angular_rate[1] = ((double)imu.gyro_LSB[1])*gyro_sens;
    imu.angular_rate[2] = ((double)imu.gyro_LSB[2])*gyro_sens;
}

void Convert_Accel(){
    const double accel_sens = 2.0/32768.0;

	memcpy(imu.accel_LSB_bytes, e_accel_data, sizeof(e_accel_data));

    imu.accel_LSB[0] = (((int16_t)imu.accel_LSB_bytes[1])<<8) + ((int16_t)imu.accel_LSB_bytes[0]);
    imu.accel_LSB[1] = -(((int16_t)imu.accel_LSB_bytes[3])<<8) - ((int16_t)imu.accel_LSB_bytes[2]);
    imu.accel_LSB[2] = -(((int16_t)imu.accel_LSB_bytes[5])<<8) - ((int16_t)imu.accel_LSB_bytes[4]);

    imu.acceleration[0] = ((double)imu.accel_LSB[0])*accel_sens;
    imu.acceleration[1] = ((double)imu.accel_LSB[1])*accel_sens;
    imu.acceleration[2] = ((double)imu.accel_LSB[2])*accel_sens;

    for (uint8_t i = 0; i < 3; i++){
		bool calculate_offset = false;
		if (imu.accel_LSB[i] > imu.accel_max[i]){
			imu.accel_max[i] = imu.accel_LSB[i];
			calculate_offset = true;
		}
		else if (imu.accel_LSB[i] < imu.accel_min[i]){
			imu.accel_min[i] = imu.accel_LSB[i];
			calculate_offset = true;
		}
		if (calculate_offset){
            imu.accel_bias_LSB[i] = (int32_t)imu.accel_max[i] + (int32_t)imu.accel_min[i];
			imu.accel_bias_LSB[i] >>= 1;
		}
	}
}

double IMU_Acceleration(uint8_t index){
    return imu.acceleration[index];
}

double IMU_Angular_Rate(uint8_t index){
    return imu.angular_rate[index];
}
