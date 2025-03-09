#include "Quadrotor.h"
#include "Environment.h"

Quadrotor::Quadrotor(double Sim_dt, double Sim_tf){
    sim_dt = Sim_dt;
    sim_tf = Sim_tf;
    Forces_NED.resize(3, 0.0);
    Moments_NED.resize(3, 0.0);
}

void Quadrotor::Run_sim(){
    double sim_t = 0.0;
    
    while(sim_t < sim_tf){
        Update_drone_states();


        sim_t += sim_dt;
    }
}

void Quadrotor::Update_drone_states(){

}