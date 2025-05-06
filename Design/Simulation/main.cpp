#include "Quadrotor.h"
#include "Sim_Time.h"

int main(){
    Sim_Time Sim_time_step = {.Seconds = 0, .MicroSeconds = 1000};
    Sim_Time Sim_finish_time = {.Seconds = 75, .MicroSeconds = 0};

    Quadrotor Drone(Sim_time_step, Sim_finish_time);
    
    Drone.Run_sim();

    return 0;
}