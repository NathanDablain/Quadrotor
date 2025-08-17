#pragma once

#include "IMU_p32.h"
#include "Time_p32.h"
#include "Environment.h"
#include "Sim_Time.h"
#include "Barometer.h"
#include "Magnetometer.h"
#include "IMU.h"

extern "C" {
    uint8_t Process_IMU_Data(IMU_Data *IMU_variables, Time Current_Time);
};
// This structure contains the current assumption of the drone states to be used for guidance and control functions
typedef struct{
    float Euler[3];
    float w[3];
    float v_dot[3];
    float P_Origin_NED[3];
} Drone_States;

class PIC32AK{
    private:
        IMU_Data IMU_variables;
        Time IMU_Timelast;
        Time IMU_Update_Time;
        void Read_Gyro();
    public:
        // LPS22H Barometer
        Barometer barometer;
        // LIS2MDL Magnetometer
        Magnetometer magnetometer;
        // IMU
        IMU imu;
        Drone_States States;
        PIC32AK();
        void Run(Environment &env, Sim_Time sim_t);
};