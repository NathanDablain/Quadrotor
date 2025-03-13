#include "Quadrotor.h"
#include "Environment.h"
#include <iostream>
#include <iomanip>

using namespace std;

Quadrotor::Quadrotor(double Sim_dt, double Sim_tf){
    sim_dt = Sim_dt;
    sim_tf = Sim_tf;
    inertia.data[0][0] = Ixx;
    inertia.data[1][1] = Iyy;
    inertia.data[2][2] = Izz;
    q = {1.0, 0.0, 0.0, 0.0};
}

void Quadrotor::Run_sim(){
    double sim_t = 0.0;
    Environment env(-97.06265, 32.79100, 0.0);
    Forces_Body = {1.0, 2.0 , 3.0};
    Moments_Body = {0.01, 0.02 , 0.03};

    while(sim_t < (sim_tf - sim_dt)){
        Update_drone_states();

        sim_t += sim_dt;
    }
    cout << setprecision(8) << "  q0:" << q.data[0] << "   q1:" << q.data[1] << "  q2:" << q.data[2] << "  q3:" << q.data[3] << endl;
}

void Quadrotor::Update_drone_states(){
    // u = q(2:4);
    // t = 2*cross(u, g);
    // g_rot = g + q(1)*t + cross(u, t);
    // Uses Runge Kutta 4th order integration to propogate momentum and NED to body quaternion
    Vec x_1, x_2, k1, k2, k3, k4, temp;
    x_1 = {v.data[0], v.data[1], v.data[2], w.data[0], w.data[1],
           w.data[2], q.data[0], q.data[1], q.data[2], q.data[3]};
    
    k1 = Differential_equation_momentum(x_1)*sim_dt;
    temp = x_1 + k1*0.5;
    k2 = Differential_equation_momentum(temp)*sim_dt;
    temp = x_1 + k2*0.5;
    k3 = Differential_equation_momentum(temp)*sim_dt;
    temp = x_1 + k3;
    k4 = Differential_equation_momentum(temp)*sim_dt;

    x_2 = x_1 + (k1 + k2*2.0 + k3*2.0 + k4)*(1.0/6.0);

    v = {x_2.data[0], x_2.data[1], x_2.data[2]};
    w = {x_2.data[3], x_2.data[4], x_2.data[5]};
    q = {x_2.data[6], x_2.data[7], x_2.data[8], x_2.data[9]};
}

Vec Quadrotor::Differential_equation_momentum(Vec &x_in){
    // Rigid body momentum equations in a rotating coordinate frame, feedback
    // incorporated in quaternion equation to maintain magnitude 1
    // v_dot = F/m - w x v
    // w_dot = I^-1*(M - w x I*w)
    // q_dot = (omega*q)+(0.5*(1-dot(q,q))*q);
    Vec x_dot, q_dot, q_loc; 
    Mat omega(4, 4);
    Vec3 v_loc = {x_in.data[0], x_in.data[1], x_in.data[2]};
    Vec3 w_loc = {x_in.data[3], x_in.data[4], x_in.data[5]};
    q_loc = {x_in.data[6], x_in.data[7], x_in.data[8], x_in.data[9]};

    omega.data = {{          0.0, -w_loc.data[0], -w_loc.data[1], -w_loc.data[2]},
                  {w_loc.data[0],            0.0,  w_loc.data[2], -w_loc.data[1]},
                  {w_loc.data[1], -w_loc.data[2],            0.0,  w_loc.data[0]},
                  {w_loc.data[2],  w_loc.data[1], -w_loc.data[0],            0.0}};

    Vec3 v_dot = (Forces_Body/mass) - w_loc.cross(v_loc);
    Vec3 w_dot = inertia.inv()*(Moments_Body - w_loc.cross(inertia*w_loc));
    q_dot = ((omega*q_loc) + (q_loc*(1.0-q_loc.dot(q_loc))))*0.5;

    x_dot = {v_dot.data[0], v_dot.data[1], v_dot.data[2], w_dot.data[0], w_dot.data[1],
             w_dot.data[2], q_dot.data[0], q_dot.data[1], q_dot.data[2], q_dot.data[3]};

    return x_dot;
}