#include <xc.h>
#include <stdint.h>
#include "motors.h"
#include "global_variables.h"
#include "time.h"

//    Motor1  RC4	RP37	PWM3L
//    Motor2  RC3	RP36	PWM3H
//    Motor3  RC5	RP38	PWM4L
//    Motor4  RC2	RP35	PWM4H

void Disable_Motors(){
    uint16_t zero_throttle[4] = {0, 0, 0, 0};
    Apply_Throttle_Batch(zero_throttle);
}


void Apply_Throttle(uint16_t throttle, uint8_t index){
    uint32_t duty_cycle = throttle + MIN_DUTY_CYCLE;
    if (duty_cycle > MAX_DUTY_CYCLE){
        duty_cycle = MAX_DUTY_CYCLE;
    }
    
    switch(index){
        case 1:
            PG3TRIGA = (duty_cycle<<4);
            PG3STATbits.UPDREQ = 1;
            break;
        case 2:
            PG3PHASE = (duty_cycle<<4);
            PG3STATbits.UPDREQ = 1;
            break;
        case 3:
            PG4TRIGA = (duty_cycle<<4);
            PG4STATbits.UPDREQ = 1;
            break;
        case 4: 
            PG4PHASE = (duty_cycle<<4);
            PG4STATbits.UPDREQ = 1;
            break;
    }
}

void Apply_Throttle_Batch(uint16_t throttle[4]){
    uint32_t duty_cycle;
    for (uint8_t i = 0; i < 4; i++){
        duty_cycle = throttle[i] + MIN_DUTY_CYCLE;
        if (duty_cycle > MAX_DUTY_CYCLE){
            duty_cycle = MAX_DUTY_CYCLE;
        }
        
        switch(i){
            case 0:
                PG3TRIGA = (duty_cycle<<4);
                break;
            case 1:
                PG3PHASE = (duty_cycle<<4);
                break;
            case 2:
                PG4TRIGA = (duty_cycle<<4);
                break;
            case 3: 
                PG4PHASE = (duty_cycle<<4);
                break;
           }
    }
    
    PG3STATbits.UPDREQ = 1;
    PG4STATbits.UPDREQ = 1;
}

void Set_All_Throttles(uint16_t throttle){
    uint32_t duty_cycle = throttle + MIN_DUTY_CYCLE;
    if (duty_cycle > MAX_DUTY_CYCLE){
        duty_cycle = MAX_DUTY_CYCLE;
    }
    
    // If update is in progress, give a chance for it to clear and then exit
    if (PG3STATbits.UPDATE == 1 || PG4STATbits.UPDATE == 1){
        Delay(1000);
    }
    if (PG3STATbits.UPDATE == 1 || PG4STATbits.UPDATE == 1){
        return;
    }
    
    PG3TRIGA = (duty_cycle<<4);
    PG3PHASE = (duty_cycle<<4);
    PG4TRIGA = (duty_cycle<<4);
    PG4PHASE = (duty_cycle<<4);
    
    PG3STATbits.UPDREQ = 1;
    PG4STATbits.UPDREQ = 1;
}

void Calibrate_Motors(){
    const int32_t drop_rate_hz = 400;
    const Time Drop_Rate       = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/drop_rate_hz};
    const Time Hold_Period     = {.seconds = 5, .tmr1_count = 0};
    uint16_t master_throttle   = MAX_THROTTLE;
    Time Hold_State            = Current_Time();
    
    // Raise Motor duty cycle to max 
    Set_All_Throttles(MAX_THROTTLE);

    while(!Compare_And_Update(Current_Time(), Hold_Period, &Hold_State));
    
    while(1){
        master_throttle -= 1;
        Set_All_Throttles(master_throttle);
        while(!Compare_And_Update(Current_Time(), Drop_Rate, &Hold_State));
        
        if (master_throttle == 0) break;
    }
    
    while(!Compare_And_Update(Current_Time(), Hold_Period, &Hold_State));

}