#include "main.h"

using namespace std;

int main(){
    double sim_dt = 0.001;
    double sim_tf = 20.0;
    Quadrotor Drone(sim_dt, sim_tf);
    
    Drone.Run_sim();

    return 0;
}