#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "Guidance_p32.h"
#include "Time_p32.h"
#include "Barometer_p32.h"
#include "IMU_p32.h"
#include "Global_Variables_p32.h"

static Guidance_State State;
static Reference_Data Reference;

void Initialize_Guidance_Machine(){
    State = Taking_Off;
    memset(&Reference, 0, sizeof(Reference));
}

void Guidance_Machine(uint32_t current_time){
    const double peak_height = 5.0;
    const uint32_t peak_hold_time = 15;
    double h = Filter_data(0, 2);

    switch(State){
        case Taking_Off:
            Reference.h_ref = peak_height;
            Reference.Euler_ref[0] = Filter_data(6, 0);
            Reference.Euler_ref[1] = Filter_data(7, 0);
            if (h > 0.35){
                State = Climbing;
            }
            break;

        case Climbing:
            Reference.h_dot_ref = 0.5;
            Reference.Inhibit_Motors = false;
            if (fabs(h - peak_height) < 0.1){
                State = Hovering;
                Reference.peak_time = current_time;
            }
            // If we are within 0.5 m of the ground, guide to the initial euler angles rather than to the gravity vector
            if (h > 0.5){
                memset(Reference.Euler_ref, 0, sizeof(Reference.Euler_ref));
            }
            break;

        case Hovering:
            Reference.h_ref = peak_height;
            Reference.Inhibit_Motors = false;
            if (current_time - Reference.peak_time > peak_hold_time){
                State = Descending;
            }
            break;

        case Descending:
            Reference.h_ref = 0.0;
            if (fabs(h) < 0.1){
                Reference.Inhibit_Motors = true;
                State = Landed;
            }
            if (h < 0.5){
                Reference.Euler_ref[0] = Filter_data(6, 0);
                Reference.Euler_ref[1] = Filter_data(7, 0);
            }
            else{
                memset(Reference.Euler_ref, 0, sizeof(Reference.Euler_ref));
            }
            break;

        case Landed:
            break;

    }
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

// cpp wrapper functions

void Initialize_Guidance_Machine_cpp(){
    Initialize_Guidance_Machine();
}

void Guidance_Machine_cpp(uint32_t current_time){
    Guidance_Machine(current_time);
}

Guidance_State Get_Guidance_State_cpp(){
    return Get_Guidance_State();
}