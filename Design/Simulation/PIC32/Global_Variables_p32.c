#include "Global_Variables_p32.h"
#include "Sim_Types.h"
#include <stdint.h>
#include <math.h>

volatile uint32_t g_seconds = 0;

const uint32_t g_tmr1_ct_in_s = 25000000;
const double g_tmr1_ct_in_s_fp = 25000000.0;

const Time g_gyro_sample_rate = {.seconds = 0, .tmr1_count = 125000};

const double g_mass = 0.545;
const double g_gravity = 9.8065;
FC_Status g_Flight_Controller_Status = Standby;