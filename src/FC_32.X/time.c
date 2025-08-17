#include <xc.h>
#include "time.h"
#include "global_variables.h"
#include <stdbool.h>

Time Current_Time(){
    Time current;
    
    current.tmr1_count = 12039021; //TMR1;
    current.seconds = g_seconds;
    
    return current;
}

Time Time_Difference(Time Time_1, Time Time_2){
    // result = Time_1 - Time_2
    Time result;
    result.seconds = Time_1.seconds - Time_2.seconds;
    result.tmr1_count = Time_1.tmr1_count - Time_2.tmr1_count;
    if (result.tmr1_count < 0){
        result.tmr1_count += g_tmr1_ct_in_s;
        result.seconds--;
    }

    return result;
}

double Time_fp(Time time){
    double result = ((double)time.seconds) + ((double)time.tmr1_count)/g_tmr1_ct_in_s_fp;
    
    return result;
}

bool Compare_And_Update(const Time Time_1, const Time Delta_t, Time* Time_2){
    bool result = false;
    Time difference = Time_Difference(Time_1, *Time_2);
    if (difference.seconds > Delta_t.seconds){
        result = true;
    }
    else if ((difference.seconds == Delta_t.seconds) && (difference.tmr1_count > Delta_t.tmr1_count)){
        result = true;
    }

    if (result){
        *Time_2 = Time_1;
    }

    return result;
}

void _ISR _T1Interrupt(void){
    ++g_seconds;
    TMR1 = 0;
}