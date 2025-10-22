#ifndef GLOBAL_VARIABLES_H
#define	GLOBAL_VARIABLES_H

#include "Time_p32.h"
#include "Sim_Types.h"
#include <stdint.h>

extern volatile uint32_t g_seconds;
extern const uint32_t g_tmr1_ct_in_s;
extern const double g_tmr1_ct_in_s_fp;
extern const Time g_gyro_sample_rate;

extern const double g_mass;
extern const double g_gravity;

extern FC_Status g_Flight_Controller_Status;

#endif