#include "Quadrotor.h"
#include "Sim_Time.h"

int main(){
    Sim_Time Sim_time_step = {.Seconds = 0, .MicroSeconds = 500};
    Sim_Time Sim_finish_time = {.Seconds = 100, .MicroSeconds = 0};

    Quadrotor Drone(Sim_time_step, Sim_finish_time);
    
    Drone.Run_sim();
    for (uint8_t i = 0; i < 6; i++){
        cout << setw(15) << Drone.Control_errors[i];
    }
    cout << endl;
        for (uint8_t i = 0; i < 6; i++){
        cout << setw(15) << Drone.Navigation_errors[i];
    }
    cout << endl;
    return 0;
}