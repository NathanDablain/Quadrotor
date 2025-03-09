#include "main.h"

using namespace std;

int main(int argc, char *argv[]){
    double sim_dt = 0.001;
    double sim_tf = 10.0;
    Quadrotor Drone(sim_dt, sim_tf);
    
    Drone.Run_sim();

    return 0;
}