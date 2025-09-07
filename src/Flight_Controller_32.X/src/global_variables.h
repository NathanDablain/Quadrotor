#ifndef GLOBAL_VARIABLES_H
#define	GLOBAL_VARIABLES_H

#include "time.h"
#include <stdint.h>

// Timing variables
extern volatile uint32_t g_seconds;
extern const int32_t g_tmr1_ct_in_s;
extern const double g_tmr1_ct_in_s_fp;

// ADC variables
extern volatile uint32_t g_adc_1s_result;
extern volatile uint32_t g_adc_4s_result;

// SPI variables
extern volatile uint8_t* g_spi1_data_out_ptr;
extern volatile uint32_t* g_spi1_reg_ptr;
extern volatile uint8_t g_spi1_pin;
extern volatile bool g_spi1_rdy_flag;
extern volatile bool* g_spi1_transfer_done_flag;

extern volatile uint8_t* g_spi2_data_out_ptr;
extern volatile uint32_t* g_spi2_reg_ptr;
extern volatile uint8_t g_spi2_pin;
extern volatile bool g_spi2_rdy_flag;
extern volatile bool* g_spi2_transfer_done_flag;

#endif

