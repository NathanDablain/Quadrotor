#ifndef GUIDANCE_H
#define	GUIDANCE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum{
    Awaiting_Guidance,
    Taking_Off,
    Climbing,
    Hovering,
    Descending,
    Landed
} Guidance_State;

typedef struct{
    double h_ref;
    double h_dot_ref;
    double Euler_ref[3];
    double P_NED_ref[3];
    bool Inhibit_Motors;
    uint32_t peak_time;
} Reference_Data;

void Initialize_Guidance_Machine();

void Guidance_Machine();

void Transition_Guidance_State(Guidance_State new_state, double peak_height);

double Reference_Altitude();

double Reference_Altitude_Dot();

double Reference_Euler(uint8_t index);

bool Inhibit_Motors();

Guidance_State Get_Guidance_State();

#endif

