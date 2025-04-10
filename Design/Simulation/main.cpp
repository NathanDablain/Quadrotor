#include "Quadrotor.h"

int main(){
    double sim_dt = 0.001;
    double sim_tf = 50.0;

    Quadrotor Drone(sim_dt, sim_tf);
    
    Drone.Run_sim();

    return 0;
}