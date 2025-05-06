#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <avr/io.h>
#include <avr/xmega.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "SPI.h"
#include "TWI.h"
#include "IMU.h"
#include "Mag.h"
#include "Bar.h"
#include "Observer.h"
#include "SSD.h"
#include "LoRa.h"
#include "Utilities.h"
#include "Motors.h"
#include "Controllers.h"
#include "FC_Types.h"

// Tracks when to check LoRa for uplink, incremented at 200 Hz
extern volatile unsigned char g_LoRa_Check_Flag;
// Tracks when to print output to SSD display
extern volatile unsigned char g_Print_Flag;
// Tracks when to sample magnetometer
extern volatile unsigned char g_MAG_Read_Flag;
// Tracks when to sample imu
extern volatile unsigned char g_Accel_Read_Flag;
// Tracks when to sample imu
extern volatile unsigned char g_Gyro_Read_Flag;
// Tracks when to sample barometer
extern volatile unsigned char g_BAR_Read_Flag;
// Tracks how often to send PPM signal to ESC, driving motor throttles
extern volatile unsigned char g_Motor_Run_Flag;
// Tracks how often to run altitude MRAC, drives desired thrust
extern volatile unsigned char g_Altitude_Control_Flag;
// Tracks how often to run guidance function
extern volatile unsigned char g_Guidance_Flag;
// Tracks when to send downlink
extern volatile unsigned char g_LoRa_Send_Flag;
// Tracks when to use accelerometer and magnetometer data to include measurements
extern volatile unsigned char g_Attitude_Observer_Update_Flag;
// Tracks when to use gyro data to make predictions
extern volatile unsigned char g_Attitude_Observer_Predict_Flag;

// Either sets or clears a bit within a bitmask depending on the input
#define SET_BIT(current_val, position, val) ((val < 1) ? current_val&(~(val<<position)) : current_val|(val<<position))
// Bit positions within navigation bitmask to signal if a given sensor is working
#define NAV_BAR_bp 1
#define NAV_IMU_bp 2
#define NAV_MAG_bp 3
#define NAV_LORA_bp 4
#define SU_SSD_bp 5
// Bit mask result when all sensors are operating correctly
#define NAV_SENSORS_bm 0x1E

unsigned char Setup();
// Initializes timers used to keep track events
void Setup_Timers();

#endif