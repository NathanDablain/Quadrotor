#include "PIC32AK.h"
#include <cstdlib>
#include <cstdbool>
#include <cstdint>
#include <cstring>
#include <cmath>


PIC32AK::PIC32AK(){

}

void PIC32AK::Run(Environment &env, Sim_Time sim_t){
    Time Current_Time = {.seconds = sim_t.Seconds, .microseconds = sim_t.MicroSeconds};

    // Sample sensors
    // if (Current_Time - IMU_Timelast > IMU_Update_Time) Read_Gyro();
    // Manage FC status

    // Run Filters

    // Run Guidance

    // Run Control
}

void PIC32AK::Read_Gyro(){	
    if (imu.gyro_drdy_flag == false) return;
    imu.gyro_drdy_flag = true;
	
	IMU_variables.gyro_output_LSB[0] = -imu.angular_rate_LSB[0];
	IMU_variables.gyro_output_LSB[1] = imu.angular_rate_LSB[1];
	IMU_variables.gyro_output_LSB[2] = -imu.angular_rate_LSB[2];

    // Process_IMU_Data(IMU_variables, )
}