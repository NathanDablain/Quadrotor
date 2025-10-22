#ifndef EXTERNAL_INTERFACE_H
#define EXTERNAL_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "Time_p32.h"
#include "Sim_Types.h"

// This interface will hold values that will mimic the interface the PIC has with external systems
// through SPI and internal registers
// The sim and the PIC will access this interface to pass information about sensor readings, radio uplinks, and motor inputs

// Generic
extern Time e_Current_Time;
extern struct States e_Output_States;

// IMU
extern uint8_t e_accel_data[6];
extern uint8_t e_gyro_data[6];
extern uint16_t e_accel_odr;
extern uint8_t e_accel_lpf_setting;
extern uint16_t e_gyro_odr;
extern uint8_t e_gyro_lpf_setting;
extern bool e_imu_settings_updated;

// MAG
extern uint8_t e_mag_data[6];
extern uint16_t e_mag_odr;
extern uint8_t e_mag_lpf_setting;
extern bool e_mag_settings_updated;

// BAR
extern uint8_t e_bar_data[3];
extern uint16_t e_bar_odr;
extern uint8_t e_bar_lpf_setting;
extern bool e_bar_low_noise_setting;
extern bool e_bar_settings_updated;

// LORA
// extern struct Uplink e_uplink;
// extern struct Downlink e_downlink;
extern uint8_t e_uplink_message[35];
extern uint8_t e_downlink_message[10];
extern bool e_uplink_ready;
extern bool e_downlink_ready;

// Motors
extern uint16_t e_throttle_commands[4];

#endif