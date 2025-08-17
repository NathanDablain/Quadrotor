#include "Quadrotor.h"
#include "Sim_Time.h"

int main(){
    Sim_Time Sim_time_step = {.Seconds = 0, .MicroSeconds = 100};
    Sim_Time Sim_finish_time = {.Seconds = 70, .MicroSeconds = 0};

    uint32_t mc_seed = 2;
    uint32_t number_of_runs = 1;
    uint32_t crashes = 0;
    for (uint32_t i = 0; i < number_of_runs; i++){
        Quadrotor Drone(Sim_time_step, Sim_finish_time, mc_seed+i);
    
        Drone.Run_sim();

        if (Drone.AVR128DB48.Flight_Controller_Status == Crashed){
            cout << "Run " << i << " ended in crash" << endl;
            crashes++;
        }
    
    }
    cout << crashes << "/" << number_of_runs << " crashed" << endl;
    // cout << setw(20) << "Control Errors: ";
    // for (uint8_t i = 0; i < 6; i++){
    //     cout << setw(15) << Drone.Control_errors[i];
    // }
    // cout << endl;
    // cout << setw(20) << "Navigation Errors: ";
    //     for (uint8_t i = 0; i < 6; i++){
    //     cout << setw(15) << Drone.Navigation_errors[i];
    // }
    // cout << endl;
    return 0;
}