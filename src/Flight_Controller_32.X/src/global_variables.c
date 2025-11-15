#include "global_variables.h"
#include "time.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <xc.h>

// Global state machine
FC_Status g_Flight_Controller_Status = Standby;

// Keeps track of seconds since end of setup
volatile uint32_t g_seconds = 0;
// How many ticks of timer1 are in one second
const int32_t g_tmr1_ct_in_s    = 12500000;
const double  g_tmr1_ct_in_s_fp = 12500000.0;

const double g_gravity = 9.8065;
const double g_mass    = 0.545;

// Result of 1S LIPO voltage level
volatile uint32_t g_adc_1s_result = 0;
// Result of 4S LIPO voltage level
volatile uint32_t g_adc_4s_result = 0;

// SPI variables
volatile uint8_t*  g_spi1_data_out_ptr;
volatile uint32_t* g_spi1_reg_ptr;
volatile uint8_t   g_spi1_pin = 0;
volatile bool      g_spi1_rdy_flag = true;
volatile bool*     g_spi1_transfer_done_flag;

volatile uint8_t*  g_spi2_data_out_ptr;
volatile uint32_t* g_spi2_reg_ptr;
volatile uint8_t   g_spi2_pin = 0;
volatile bool      g_spi2_rdy_flag = true;
volatile bool*     g_spi2_transfer_done_flag;

// I2C1 variables
volatile bool    g_i2c1_rdy_flag = true;
volatile uint8_t g_i2c1_slave_address;
volatile bool    g_oled_fail_flag = false;

// CCP/CCT variables
volatile bool g_air_filter_predict_flag      = false;
volatile bool g_ground_filter_predict_flag   = false;
volatile bool g_gyro_sample_flag             = false;
volatile bool g_accel_sample_flag            = false;
volatile bool g_altitude_filter_predict_flag = false;
volatile bool g_ground_filter_update_flag    = false;
volatile bool g_magnetometer_sample_flag     = false;
volatile bool g_altitude_filter_update_flag  = false;
volatile bool g_barometer_sample_flag        = false;
volatile bool g_air_filter_update_flag       = false;
volatile bool g_lora_update_flag             = false;
volatile bool g_moment_control_outer_flag    = false;
volatile bool g_moment_control_inner_flag    = false;
volatile bool g_thrust_control_middle_flag   = false;
volatile bool g_thrust_control_inner_flag    = false;
volatile bool g_motor_update_flag            = false;
volatile bool g_barometer_filter_flag        = false;
volatile bool g_magnetometer_filter_flag     = false;

// Derived from CCP/CCT variables
volatile bool g_run_safety_check_flag        = false;
