#include "External_Interface.h"
#include <stdint.h>
#include <string.h>

void Reset_External_Interface(){
    memset(&e_Current_Time, 0, sizeof(e_Current_Time));
    memset(&e_Output_States, 0, sizeof(e_Output_States));
    memset(e_accel_data, 0, sizeof(e_accel_data));
    memset(e_gyro_data, 0, sizeof(e_gyro_data));
    e_accel_odr = 1;
    e_accel_lpf_setting = 4;
    e_gyro_odr = 1;
    e_gyro_lpf_setting = 4;
    e_imu_settings_updated = false;
    memset(e_mag_data, 0, sizeof(e_mag_data));
    e_mag_odr = 1;
    e_mag_lpf_setting = 0;
    e_bar_low_noise_setting = false;
    e_mag_settings_updated = false;
    memset(e_bar_data, 0, sizeof(e_bar_data));
    e_bar_odr = 1;
    e_bar_lpf_setting = 2;
    e_bar_settings_updated = false;
    memset(e_uplink_message, 0, sizeof(e_uplink_message));
    memset(e_downlink_message, 0, sizeof(e_downlink_message));
    e_uplink_ready = false;
    e_downlink_ready = false;
    memset(e_throttle_commands, 0, sizeof(e_throttle_commands));
}

// Generic
Time e_Current_Time;
struct States e_Output_States;

// IMU
uint8_t e_accel_data[6];
uint8_t e_gyro_data[6];
uint16_t e_accel_odr;
uint8_t e_accel_lpf_setting;
uint16_t e_gyro_odr;
uint8_t e_gyro_lpf_setting;
bool e_imu_settings_updated;

// MAG
uint8_t e_mag_data[6];
uint16_t e_mag_odr;
uint8_t e_mag_lpf_setting;
bool e_mag_settings_updated;

// BAR
uint8_t e_bar_data[3];
uint16_t e_bar_odr;
uint8_t e_bar_lpf_setting;
bool e_bar_low_noise_setting;
bool e_bar_settings_updated;

// LORA
// struct Uplink e_uplink;
// struct Downlink e_downlink;
uint8_t e_uplink_message[35];
uint8_t e_downlink_message[10];
bool e_uplink_ready;
bool e_downlink_ready;

// Motors
uint16_t e_throttle_commands[4];