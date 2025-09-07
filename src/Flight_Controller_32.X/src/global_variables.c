#include "global_variables.h"
#include "time.h"
#include <stddef.h>
#include <stdint.h>
#include <xc.h>

// Keeps track of seconds since end of setup
volatile uint32_t g_seconds = 0;
// How many ticks of timer1 are in one second
const int32_t g_tmr1_ct_in_s = 12500000;
const double g_tmr1_ct_in_s_fp = 12500000.0;

// Result of 1S LIPO voltage level
volatile uint32_t g_adc_1s_result = 0;
// Result of 4S LIPO voltage level
volatile uint32_t g_adc_4s_result = 0;

// SPI variables
volatile uint8_t* g_spi1_data_out_ptr;
volatile uint32_t* g_spi1_reg_ptr;
volatile uint8_t g_spi1_pin = 0;
volatile bool g_spi1_rdy_flag = true;
volatile bool *g_spi1_transfer_done_flag;

volatile uint8_t* g_spi2_data_out_ptr;
volatile uint32_t* g_spi2_reg_ptr;
volatile uint8_t g_spi2_pin = 0;
volatile bool g_spi2_rdy_flag = true;
volatile bool *g_spi2_transfer_done_flag;
