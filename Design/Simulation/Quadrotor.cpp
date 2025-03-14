#include "Quadrotor.h"
#include "Environment.h"
#include "Coordinate_Frames.hpp"
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
    // Forces_Body = {1.0, 2.0 , 3.0};
    // Moments_Body = {0.01, 0.02 , 0.03};

    while(sim_t < (sim_tf - sim_dt)){
        Update_drone_forces_moments(env.gravity, env.ground_stiffness, env.ground_damping);
        
        Update_drone_states();

        sim_t += sim_dt;
    }
    cout << setprecision(8) << "  P0:" << Position_NED.data[0] << "   P1:" << Position_NED.data[1] << "  P2:" << Position_NED.data[2] << endl;
}

void Quadrotor::Update_drone_forces_moments(double gravity, double ground_stiffness, double ground_damping){
    // Update drone forces and moments, the following effects are considered:
    // -> Gravity
    // -> Motors
    // -> Wind
    // -> Ground

    // Gravity force, dependent on initial LLA position
    Vec3 g_vec_NED = {0.0, 0.0, gravity};
    Vec3 g_force_Body = NED2Body(g_vec_NED, q)*mass;
    // Motor force and moment
    Vec motor_thrusts;
    motor_thrusts = {Motors[0].Get_motor_thrust(), Motors[1].Get_motor_thrust(),
                     Motors[2].Get_motor_thrust(), Motors[3].Get_motor_thrust()};
    Vec3 motor_force_Body = {0.0, 0.0, -motor_thrusts.magnitude()};
    // Assume that:
    // -> Back motor (0) produces negative pitching torque and negative yawing torque
    // -> Left motor (1) produces positive rolling torque and positive yawing torque
    // -> Front motor (2) produces positive pitching torque and negative yawing torque
    // -> Right motor (3) produces negative rolling torque and positive yawing torque
    Vec3 motor_moment_Body = {
        length*(motor_thrusts.data[1] - motor_thrusts.data[3]),
        length*(motor_thrusts.data[2] - motor_thrusts.data[0]),
        length*(Motors[1].Get_motor_torque() + Motors[3].Get_motor_torque() - Motors[0].Get_motor_torque() - Motors[2].Get_motor_torque())};
    // Ground force
    // Model the ground as a lumped parameter model, it has some stiffness and some damping
    Vec3 ground_forces_Body;
    if (Position_NED.data[2] >= 0){
        Vec3 to_rotate = {0.0, 0.0, Position_NED.data[2]};
        Vec3 Position_Body = NED2Body(to_rotate, q);
        ground_forces_Body = Position_Body*-ground_stiffness - v*ground_damping;
    }
    Forces_Body = g_force_Body + motor_force_Body + ground_forces_Body; // + wind_force_Body;
    Moments_Body = motor_moment_Body; // + wind_moment_Body;
}

void Quadrotor::Update_drone_states(){
    // Uses Runge Kutta 4th order integration to propogate momentum, NED to body quaternion, and NED position
    Vec x_1, x_2, k1, k2, k3, k4, temp;
    x_1 = {v.data[0], v.data[1], v.data[2],
           w.data[0], w.data[1], w.data[2], 
           q.data[0], q.data[1], q.data[2], q.data[3],
           Position_NED.data[0], Position_NED.data[1], Position_NED.data[2]};
    
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
    Position_NED = {x_2.data[10], x_2.data[11], x_2.data[12]};

    Euler = {atan2( 2.0*(q.data[0]*q.data[1] + q.data[2]*q.data[3]) , 1.0 - 2.0*(pow(q.data[1],2) + pow(q.data[2],2)) ),
             asin( 2.0*(q.data[0]*q.data[2] - q.data[1]*q.data[3]) ),
             atan2( 2.0*(q.data[0]*q.data[3] + q.data[1]*q.data[2]) , 1.0 - 2.0*(pow(q.data[2],2) + pow(q.data[3],2)) )};
}

Vec Quadrotor::Differential_equation_momentum(Vec &x_in){
    // Rigid body momentum equations in a rotating coordinate frame, feedback
    // incorporated in quaternion equation to maintain magnitude 1
    // v_dot = F/m - w x v
    // w_dot = I^-1*(M - w x I*w)
    // q_dot = (omega*q)+(0.5*(1-dot(q,q))*q);
    // P_dot = Body2NED(v, q)
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
    Vec3 P_dot = Body2NED(v_loc, q_loc);

    x_dot = {v_dot.data[0], v_dot.data[1], v_dot.data[2],
             w_dot.data[0], w_dot.data[1], w_dot.data[2],
             q_dot.data[0], q_dot.data[1], q_dot.data[2], q_dot.data[3],
             P_dot.data[0], P_dot.data[1], P_dot.data[2]};

    return x_dot;
}