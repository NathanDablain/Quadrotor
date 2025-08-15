#include <xc.h>
#include "time.h"
#include <stdbool.h>

volatile uint32_t g_seconds = 0;

Time Current_Time(){
    Time current;
    
    current.microseconds = TMR1/25;
    current.seconds = g_seconds;
    
    return current;
}

Time Time_Difference(Time Time_1, Time Time_2){
    // result = Time_1 - Time_2
    Time result;
    result.seconds = Time_1.seconds - Time_2.seconds;
    result.microseconds = Time_1.microseconds - Time_2.microseconds;
    if (result.microseconds < 0){
        result.microseconds += US_IN_S;
        result.seconds--;
    }

    return result;
}

double Time_fp(Time time){
    double result = ((double)time.seconds) + ((double)time.microseconds)/1e6;
    
    return result;
}

bool Compare_And_Update(const Time Time_1, Time* Time_2){
    bool result = false;
    if (Time_1.seconds > Time_2->seconds){
        result = true;
    }
    else if ((Time_1.seconds == Time_2->seconds) && (Time_1.microseconds > Time_2->microseconds)){
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