#pragma once

#include <cstdint>
#include <math.h>
#include <cstdlib>
#include <string.h>
#include "Controllers.h"
#include "Environment.h"
#include "Sim_Time.h"
#include "Sim_Types.h"
#include "Barometer.h"
#include "Magnetometer.h"
#include "IMU.h"

#define BAR_SENS 1.0/40.96 // Pa/LSB
#define BAR_WINDOW_SIZE 16
#define BAR_TB 288.15 // Standard temperature at sea level (K)
#define BAR_G 9.8065 // Acceleration due to gravity (m/s^2)
#define BAR_LB -0.0065 // Standard temperature lapse rate (K/m)
#define BAR_PB 101325.0 // Standard static pressure at sea level (Pa)
#define BAR_R 8.31432 // Universal gas constant (J/(mol-K))S
#define BAR_M 0.0289644 // Molar mass of air (kg/mol)
#define BAR_MAX_CAL_DIFF 5.0

#define ACCEL_WINDOW_SIZE 8 // Size of FIR window
#define GYRO_WINDOW_SIZE 2
#define GYRO_SENS 500.0/32768.0
#define ACCEL_SENS 2.0/32768.0
#define W_CAL_LIMIT 500

#define MAG_CAL_TIMEOUT 5

// Increasing the gain increases the trust on the model(gyro), and decreasing it increases the trust on the measurement (accel & mag)
#define OBSERVER_GAIN 0.05f
// Time step between observer predictions
#define OBSERVER_DT 0.0025f
// Angle at which to stop making predictions and rely entirely on measurements because of pitch discontinuity
#define OBSERVER_GIMBAL_LOCK_CHECK (5.0f*D2R)

float Height_Bar(uint32_t pressure_LSB);

class MCU{
    private:
        const Sim_Time rtc_rate = {.Seconds = 1, .MicroSeconds = 0};
        const Sim_Time tcb0_rate = {.Seconds = 0, .MicroSeconds = 5000};
        const Sim_Time tcb1_rate = {.Seconds = 0, .MicroSeconds = 2400};
        const Sim_Time tcb2_rate = {.Seconds = 0, .MicroSeconds = 600};

        Sim_Time rtc_timelast = {0};
        Sim_Time tcb0_timelast = {0};
        Sim_Time tcb1_timelast = {0};
        Sim_Time tcb2_timelast = {0};
        Sim_Time gps_timelast = {0};
        Sim_Time ready_time = {0};

        uint32_t seconds = 0;
        uint8_t LoRa_Read_Flag = 0;
        uint8_t Motor_Run_Flag = 0;
        uint8_t BAR_Read_Flag = 0;
        uint8_t Attitude_Observer_Predict_Flag = 0;
        uint8_t Attitude_Observer_Update_Flag = 0;
        uint8_t MAG_Read_Flag = 0;
        uint8_t Accel_Read_Flag = 0; 
        uint8_t Gyro_Read_Flag = 0; 
        uint8_t GPS_Read_Flag = 0;
        uint8_t Altitude_Control_Flag = 0; 
        uint8_t LQR_Flag = 0;
        uint8_t Guidance_Flag = 0;
        uint8_t reset = 0;
        uint16_t motor_throttles[4] = {0};
        Calibration_Data cal_data = {0};
        Uplink up_link;
        // Flag to enable guidance and control functions in MCU after calibration
        bool MCU_Cal_Flag = false;
        void Run_Timers(Sim_Time sim_t);
        void Read_Bar();
        void Read_LoRa(Environment &env);
        void Read_Mag();
        void Read_Gyro();
        void Read_Accel();
        void Calibrate_Bar();
        void Calibrate_Mag();
        void Calibrate_Gyro();
    public:
        FC_Status Flight_Controller_Status = Standby;
        // Desired state of the drone, should mirror its counterpart in src/Flight_Controller
        States Reference = {0};
        // Current state of the drone according to the MCU, should mirror the performance in src/Flight_Controller
        States mcu = {0};
        // LPS22H Barometer
        Barometer barometer;
        // LIS2MDL Magnetometer
        Magnetometer magnetometer;
        // IMU
        IMU imu;
        // Desired thrust by MCU
        float Desired_Thrust = 0;
        // Desired moments by MCU
        float Desired_Moments[3] = {0};
        // Desired position from ground controller
        float Desired_Position_NED[3] = {0};
        // For testing
        float Desired_Euler[3] = {0};

        uint16_t mapped_throttle_commands[4] = {0};
        void Run(Environment &env, Sim_Time sim_t);
        void Run_Motors(uint16_t throttles[4]);
        void Observer_Update();
        void Observer_Predict();
        void Run_Guidance();
};