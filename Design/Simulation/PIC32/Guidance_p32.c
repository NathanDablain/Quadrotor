#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "Guidance_p32.h"
#include "Lora_p32.h"
#include "Navigation_p32.h"
#include "Time_p32.h"
#include "Barometer_p32.h"
#include "IMU_p32.h"
#include "Global_Variables_p32.h"

static Guidance_State State;
static Reference_Data Reference;

void Initialize_Guidance_Machine(){
    State = Awaiting_Guidance;
    memset(&Reference, 0, sizeof(Reference));
}

void Guidance_Machine(){
    double h = Altitude_Filter_data(0);
    double h_dot = Altitude_Filter_data(1);
    double peak_height = Uplink_Altitude();

    switch(State){
        case Awaiting_Guidance:
            Transition_Guidance_State(Taking_Off, peak_height);
            break;
        case Taking_Off:
            if (h_dot > 0.20){
                Transition_Guidance_State(Climbing, peak_height);
            }
            break;

        case Climbing:
            if (fabs(h - peak_height) < 0.1){
                Transition_Guidance_State(Hovering, peak_height);
            }
            break;

        case Hovering:
            if (g_Flight_Controller_Status == Landing){
                Transition_Guidance_State(Descending, peak_height);
            }
            break;

        case Descending:
            if (fabs(h) < 0.05){
                Transition_Guidance_State(Landed, peak_height);
            }
            
            memset(Reference.Euler_ref, 0, sizeof(Reference.Euler_ref));
            
            if (g_Flight_Controller_Status == Flying){
                Transition_Guidance_State(Climbing, peak_height);
            }
            break;

        case Landed:
            if (g_Flight_Controller_Status == Ready){
                Transition_Guidance_State(Taking_Off, peak_height);
            }
            break;

    }
}

void Transition_Guidance_State(Guidance_State new_state, double peak_height){
    // const char *renum_guid[] = {"Awaiting_Guidance", "Taking_Off", "Climbing", "Hovering", "Descending", "Landed"};
    // printf("Guidance state transitioned to %s at %f\n", renum_guid[new_state], Time_fp(Current_Time()));
    switch(new_state){
        case Awaiting_Guidance:
            Reference.Inhibit_Motors = true;
        case Taking_Off:
            Reference.Inhibit_Motors = false;
            Reference.h_ref = peak_height;
            Reference.Euler_ref[0] = Ground_Filter_data(6);
            Reference.Euler_ref[1] = Ground_Filter_data(7);
            Reference.Euler_ref[2] = Ground_Filter_data(8);
            break;

        case Climbing:
            Reference.Inhibit_Motors = false;
            Reference.h_ref = peak_height;
            Reference.Euler_ref[0] = 0.0;
            Reference.Euler_ref[1] = 0.0;
            break;

        case Hovering:
            Reference.Inhibit_Motors = false;
            Reference.h_ref = peak_height;
            Reference.Euler_ref[0] = 0.0;
            Reference.Euler_ref[1] = 0.0;
            break;

        case Descending:
            Reference.Inhibit_Motors = false;
            Reference.h_ref = 0.0;
            Reference.Euler_ref[0] = 0.0;
            Reference.Euler_ref[1] = 0.0;
            break;

        case Landed:
            Reference.Inhibit_Motors = true;
            break;

    }

    State = new_state;
}

double Reference_Altitude(){
    return Reference.h_ref;
}

double Reference_Altitude_Dot(){
    return Reference.h_dot_ref;
}

double Reference_Euler(uint8_t index){
    return Reference.Euler_ref[index];
}

bool Inhibit_Motors(){
    return Reference.Inhibit_Motors;
}

Guidance_State Get_Guidance_State(){
    return State;
}