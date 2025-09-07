#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdint.h>
#include <stdbool.h>

// Rather than use microseconds and have to divide an integer by 25, keep track of
// the timer 1 count instead. This is better for comparing times, if performing calculations
// then convert to microseconds
typedef struct{
    int32_t seconds;
    int32_t tmr1_count;
} Time;

typedef struct{
    Time imu_clk;
    Time mag_clk;
    Time bar_clk;
    Time lora_clk;
    Time gps_clk;
} SEQUENCER;

Time Current_Time();

Time Time_Difference(Time Time_1, Time Time_2);

double Time_fp(Time time);

bool Compare_And_Update(const Time Time_1, const Time Delta_t, Time* Time_2);

void Delay(uint64_t length);

#endif