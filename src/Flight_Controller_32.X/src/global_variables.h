#ifndef GLOBAL_VARIABLES_H
#define	GLOBAL_VARIABLES_H

#include "time.h"
#include "system_types.h"
#include <stdint.h>
#include <stdbool.h>

// Global state machine
extern FC_Status g_Flight_Controller_Status;

// Timing variables
extern volatile uint32_t g_seconds;
extern const    int32_t  g_tmr1_ct_in_s;
extern const    double   g_tmr1_ct_in_s_fp;

// Math variables
// Converts degrees to radians and vice versa
#define D2R (M_PI/180.0)
#define R2D (1.0/D2R)

extern const double g_gravity;
extern const double g_mass;
// ADC variables
extern volatile uint32_t g_adc_1s_result;
extern volatile uint32_t g_adc_4s_result;

// SPI variables
extern volatile uint8_t*  g_spi1_data_out_ptr;
extern volatile uint32_t* g_spi1_reg_ptr;
extern volatile uint8_t   g_spi1_pin;
extern volatile bool      g_spi1_rdy_flag;
extern volatile bool*     g_spi1_transfer_done_flag;

extern volatile uint8_t*  g_spi2_data_out_ptr;
extern volatile uint32_t* g_spi2_reg_ptr;
extern volatile uint8_t   g_spi2_pin;
extern volatile bool      g_spi2_rdy_flag;
extern volatile bool*     g_spi2_transfer_done_flag;

// I2C variables
extern volatile bool    g_i2c1_rdy_flag;
extern volatile uint8_t g_i2c1_slave_address;
extern volatile bool    g_oled_fail_flag;

// CCP/CCT variables
extern volatile bool g_air_filter_predict_flag;
extern volatile bool g_ground_filter_predict_flag;
extern volatile bool g_gyro_sample_flag;
extern volatile bool g_accel_sample_flag;
extern volatile bool g_altitude_filter_predict_flag;
extern volatile bool g_ground_filter_update_flag;
extern volatile bool g_magnetometer_sample_flag;
extern volatile bool g_altitude_filter_update_flag;
extern volatile bool g_barometer_sample_flag;
extern volatile bool g_air_filter_update_flag;
extern volatile bool g_lora_update_flag;
extern volatile bool g_moment_control_outer_flag;
extern volatile bool g_moment_control_inner_flag;
extern volatile bool g_thrust_control_middle_flag;
extern volatile bool g_thrust_control_inner_flag;
extern volatile bool g_motor_update_flag;
extern volatile bool g_barometer_filter_flag;
extern volatile bool g_magnetometer_filter_flag;

// Derived from CCP/CCT variables
extern volatile bool g_run_safety_check_flag;

#endif

