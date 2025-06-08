#include "Quadrotor.h"

using namespace std;

Quadrotor::Quadrotor(Sim_Time Sim_dt, Sim_Time Sim_tf){
    sim_dt = Sim_dt;
    sim_tf = Sim_tf;
    sim_t.MicroSeconds = 0;
    sim_t.Seconds = 0;
    cal_start_time = {.Seconds = 5, .MicroSeconds = 0};
    inertia.data[0][0] = Ixx;
    inertia.data[1][1] = Iyy;
    inertia.data[2][2] = Izz;
    q.data.resize(4, (double)0.0);
    q.data = {1.0, 0.0, 0.0, 0.0};
    Motors[0].deadzone = 100;
    Motors[1].deadzone = 100;
    Motors[2].deadzone = 100;
    Motors[3].deadzone = 100;
    AVR128DB48.barometer.Initialize(75, 1, Bar_Mode_Bypass);
    AVR128DB48.magnetometer.Initialize(50);
    AVR128DB48.imu.Initialize(416, 52, 8);
    AVR128DB48.Reference.Position_NED[2] = -5.0;

    if (log_flag){
        // log_sim.open("Sim_log.txt");
        // log_mcu.open("MCU_log.txt");
    }

}

void Quadrotor::Calculate_errors(){
    if ((AVR128DB48.Flight_Controller_Status == Flying)||(AVR128DB48.Flight_Controller_Status == Landing)){
        for (uint8_t i = 0; i < 3; i++){
            Control_errors[i] += fabs(AVR128DB48.Desired_Euler[i] - Euler.data[i])*sim_dt.Time_fp();
            Control_errors[3+i] += fabs(AVR128DB48.Desired_Position_NED[i] - Position_NED.data[i])*sim_dt.Time_fp();
            Navigation_errors[i] += fabs(Euler.data[i] - AVR128DB48.mcu.Euler[i])*sim_dt.Time_fp();
            Navigation_errors[3+i] += fabs(Position_NED.data[i] - AVR128DB48.mcu.Position_NED[i])*sim_dt.Time_fp();
        }
    }
}

void Quadrotor::Run_sim(){
    Environment env(-97.06265, 32.79100, 0.0, sim_dt);
    if (error_flag){
        memset(Control_errors, 0.0, sizeof(Control_errors));
        memset(Navigation_errors, 0.0, sizeof(Navigation_errors));
    }

    while(sim_t <= sim_tf){
        env.Update(Position_NED, q, v);

        Run_Sensors(env);

        Manage_FC_Status();

        AVR128DB48.Run(env, sim_t);

        Update_drone_forces_moments(env);
        
        Update_drone_states();

        if (log_flag) Log_data(env);

        if (error_flag) Calculate_errors();

        sim_t += sim_dt;
    }
    if (log_flag){
        // log_sim.close();
        // log_mcu.close();
    }
    if (plot_flag) system("gnuplot plotter.plt");
    // cout << setprecision(8) << "  P0:" << Position_NED.data[0] << "   P1:" << Position_NED.data[1] << "  P2:" << Position_NED.data[2] << endl;
}

void Quadrotor::Manage_FC_Status(){
    if (sim_t == cal_start_time){
        AVR128DB48.Flight_Controller_Status = Calibrating;
    }
    if ((sim_t.Seconds - cal_start_time.Seconds >= 2)&&(AVR128DB48.Flight_Controller_Status == Calibrating)){
        w.data[0] = 0.2;
        w.data[1] = -0.04;
        w.data[2] = 0.2;
    }
    if (AVR128DB48.Flight_Controller_Status == Ready){
        w.data[0] = 0; w.data[1] = 0; w.data[2] = 0;
        q.data[0] = 1; q.data[1] = 0; q.data[2] = 0; q.data[3] = 0;
    }
}

void Quadrotor::Run_Sensors(Environment &env){
    AVR128DB48.barometer.Sample(env, sim_t);
    AVR128DB48.magnetometer.Sample(env, sim_t);
    AVR128DB48.imu.Sample_Acc(env, q, sim_t);
    AVR128DB48.imu.Sample_Gyr(env, w, sim_t);
}

void Quadrotor::Log_data(Environment &env){
    const Sim_Time sim_init = {.Seconds = 0, .MicroSeconds = 0};
    const Sim_Time log_rate = {.Seconds = 0, .MicroSeconds = 10000};
    static Sim_Time Time_last;
    if (sim_t == sim_init){
        log_sim.open("Sim_log.txt");
        LOG_DATA("Time", log_sim);
        LOG_DATA("P_n", log_sim);
        LOG_DATA("P_e", log_sim);
        LOG_DATA("Height", log_sim);
        LOG_DATA("Pressure", log_sim);
        LOG_DATA("Roll", log_sim);
        LOG_DATA("Pitch", log_sim);
        LOG_DATA("Yaw", log_sim);
        LOG_DATA("w_x", log_sim);
        LOG_DATA("w_y", log_sim);
        LOG_DATA("w_z", log_sim);
        LOG_DATA("v_x", log_sim);
        LOG_DATA("v_y", log_sim);
        LOG_DATA("v_z", log_sim);
        LOG_DATA("Back Motor Thrust", log_sim);
        LOG_DATA("Left Motor Thrust", log_sim);
        LOG_DATA("Right Motor Thrust", log_sim);
        LOG_DATA("Front Motor Thrust", log_sim);
        LOG_DATA("Moment X", log_sim);
        LOG_DATA("Moment Y", log_sim);
        LOG_DATA("Moment Z", log_sim);
        log_sim << endl;
        log_sim.close();
        log_mcu.open("MCU_log.txt");
        LOG_DATA("Time", log_mcu);
        LOG_DATA("FC Status", log_mcu);
        LOG_DATA("P_n", log_mcu);
        LOG_DATA("P_e", log_mcu);
        LOG_DATA("Height", log_mcu);
        LOG_DATA("Pressure", log_mcu);
        LOG_DATA("Roll", log_mcu);
        LOG_DATA("Pitch", log_mcu);
        LOG_DATA("Yaw", log_mcu);
        LOG_DATA("w_x", log_mcu);
        LOG_DATA("w_y", log_mcu);
        LOG_DATA("w_z", log_mcu);
        LOG_DATA("Roll Desired", log_mcu);
        LOG_DATA("Pitch Desired", log_mcu);
        LOG_DATA("Yaw Desired", log_mcu);
        LOG_DATA("M_x Desired", log_mcu);
        LOG_DATA("M_y Desired", log_mcu);
        LOG_DATA("M_z Desired", log_mcu);
        LOG_DATA("Thrust Desired", log_mcu);
        LOG_DATA("P_n Desired", log_mcu);
        LOG_DATA("P_e Desired", log_mcu);
        LOG_DATA("P_d Desired", log_mcu);
        log_mcu << endl;
        log_mcu.close();
    }
    if (sim_t - Time_last >= log_rate){
        Time_last = sim_t;
        log_sim.open("Sim_log.txt", ios::app);
        log_sim << setprecision(8);
        LOG_DATA(sim_t.Time_fp(), log_sim);
        LOG_VEC3(Position_NED, log_sim);
        LOG_DATA(env.pressure, log_sim);
        LOG_VEC3(Euler, log_sim);
        LOG_VEC3(w, log_sim);
        LOG_VEC3(v, log_sim);
        LOG_DATA(Motors[0].Get_motor_thrust(), log_sim);
        LOG_DATA(Motors[1].Get_motor_thrust(), log_sim);
        LOG_DATA(Motors[2].Get_motor_thrust(), log_sim);
        LOG_DATA(Motors[3].Get_motor_thrust(), log_sim);
        LOG_VEC3(Moments_Body, log_sim);
        log_sim << endl;
        log_sim.close();
        log_mcu.open("MCU_log.txt", ios::app);
        log_mcu << setprecision(8);
        LOG_DATA(sim_t.Time_fp(), log_mcu);
        LOG_DATA(AVR128DB48.Flight_Controller_Status, log_mcu);
        LOG_ARR3(AVR128DB48.mcu.Position_NED, log_mcu);
        LOG_DATA(AVR128DB48.mcu.pressure, log_mcu);
        LOG_ARR3(AVR128DB48.mcu.Euler, log_mcu);
        LOG_ARR3(AVR128DB48.mcu.w, log_mcu);
        LOG_ARR3(AVR128DB48.Desired_Euler, log_mcu);
        LOG_ARR3(AVR128DB48.Desired_Moments, log_mcu);
        LOG_DATA(AVR128DB48.Desired_Thrust, log_mcu);
        LOG_ARR3(AVR128DB48.Desired_Position_NED, log_mcu);
        log_mcu << endl;
        log_mcu.close();
    }
}

void Quadrotor::Update_drone_forces_moments(Environment &env){
    // Update drone forces and moments, the following effects are considered:
    // -> Gravity
    // -> Motors
    // -> Wind
    // -> Ground

    // Gravity force, dependent on initial LLA position
    Vec3 g_vec_NED = {0.0, 0.0, env.gravity};
    Vec3 g_force_Body = NED2Body(g_vec_NED, q)*mass;
    // Motor force and moment
    Vec motor_thrusts(4);
    for (uint8_t i = 0; i < 4; i++){
        Motors[i].Throttle = AVR128DB48.mapped_throttle_commands[i];
        Motors[i].Update_speed();
    }
    motor_thrusts.data = {Motors[0].Get_motor_thrust(), Motors[1].Get_motor_thrust(),
                     Motors[2].Get_motor_thrust(), Motors[3].Get_motor_thrust()};
    Vec3 motor_force_Body = {0.0, 0.0, -motor_thrusts.magnitude()};
    // Assume that:
    // -> Back motor (0) produces negative pitching torque and negative yawing torque
    // -> Left motor (1) produces positive rolling torque and positive yawing torque
    // -> Right motor (2) produces negative rolling torque and positive yawing torque
    // -> Front motor (3) produces positive pitching torque and negative yawing torque
    Vec3 moment_Noise = {0.0,0.0,0.0};
    const double moment_max_noise = 0.01;
    const double moment_noise_sens = moment_max_noise/32767.0;
    if (AVR128DB48.Flight_Controller_Status == Flying || AVR128DB48.Flight_Controller_Status == Landing){
        for (uint8_t i = 0; i < 3; i++){
            double random_noise = 2.0*rand()*moment_noise_sens;
            moment_Noise.data[i] = (random_noise>moment_max_noise)?(random_noise-moment_max_noise):(random_noise);
        }
    }
    Vec3 motor_moment_Body = {
        length_l_r*(motor_thrusts.data[1] - motor_thrusts.data[2]),
        length_f_b*(motor_thrusts.data[3] - motor_thrusts.data[0]),
        Motors[1].Get_motor_torque() + Motors[2].Get_motor_torque() - Motors[0].Get_motor_torque() - Motors[3].Get_motor_torque()};
    // Ground force
    // Model the ground as a lumped parameter model, it has some stiffness and some damping
    Vec3 ground_forces_NED;
    Vec3 ground_forces_Body;
    Vec3 Velocity_NED = Body2NED(v, q);
    if (Position_NED.data[2] >= 0){
        ground_forces_NED = {0.0, 0.0, -Position_NED.data[2]*env.ground_stiffness + -Velocity_NED.data[2]*env.ground_damping};
        ground_forces_Body = NED2Body(ground_forces_NED, q);
    }
    else{
        ground_forces_Body = {0.0, 0.0, 0.0};
    }
    Forces_Body = g_force_Body + motor_force_Body + ground_forces_Body; // + wind_force_Body;
    Moments_Body = motor_moment_Body + moment_Noise; // + wind_moment_Body;
}

void Quadrotor::Update_drone_states(){
    // Uses Runge Kutta 4th order integration to propogate momentum, NED to body quaternion, and NED position
    Vec x_1(13), x_2(13), k1(13), k2(13), k3(13), k4(13), temp(13);
    x_1.data = {v.data[0], v.data[1], v.data[2],
                w.data[0], w.data[1], w.data[2], 
                q.data[0], q.data[1], q.data[2], q.data[3],
                Position_NED.data[0], Position_NED.data[1], Position_NED.data[2]};
    
    k1 = Differential_equation_momentum(x_1)*sim_dt.Time_fp();
    temp = x_1 + k1*0.5;
    k2 = Differential_equation_momentum(temp)*sim_dt.Time_fp();
    temp = x_1 + k2*0.5;
    k3 = Differential_equation_momentum(temp)*sim_dt.Time_fp();
    temp = x_1 + k3;
    k4 = Differential_equation_momentum(temp)*sim_dt.Time_fp();

    x_2 = x_1 + (k1 + k2*2.0 + k3*2.0 + k4)*(1.0/6.0);

    v = {x_2.data[0], x_2.data[1], x_2.data[2]};
    w = {x_2.data[3], x_2.data[4], x_2.data[5]};
    q.data = {x_2.data[6], x_2.data[7], x_2.data[8], x_2.data[9]};
    Position_NED = {x_2.data[10], x_2.data[11], x_2.data[12]};

    Euler = {atan2( 2.0*(q.data[0]*q.data[1] + q.data[2]*q.data[3]) , 1.0 - 2.0*(pow(q.data[1],2) + pow(q.data[2],2)) ),
             asin( 2.0*(q.data[0]*q.data[2] - q.data[1]*q.data[3]) ),
             atan2( 2.0*(q.data[0]*q.data[3] + q.data[1]*q.data[2]) , 1.0 - 2.0*(pow(q.data[2],2) + pow(q.data[3],2)) )};
}

Vec Quadrotor::Differential_equation_momentum(Vec x_in){
    // Rigid body momentum equations in a rotating coordinate frame, feedback
    // incorporated in quaternion equation to maintain magnitude 1
    // v_dot = F/m - w x v
    // w_dot = I^-1*(M - w x I*w)
    // q_dot = (omega*q)+(0.5*(1-dot(q,q))*q);
    // P_dot = Body2NED(v, q)
    Vec x_dot(13), q_dot(4), q_loc(4); 
    Mat omega(4, 4);
    Vec3 v_loc = {x_in.data[0], x_in.data[1], x_in.data[2]};
    Vec3 w_loc = {x_in.data[3], x_in.data[4], x_in.data[5]};
    q_loc.data = {x_in.data[6], x_in.data[7], x_in.data[8], x_in.data[9]};

    omega.data = {{          0.0, -w_loc.data[0], -w_loc.data[1], -w_loc.data[2]},
                  {w_loc.data[0],            0.0,  w_loc.data[2], -w_loc.data[1]},
                  {w_loc.data[1], -w_loc.data[2],            0.0,  w_loc.data[0]},
                  {w_loc.data[2],  w_loc.data[1], -w_loc.data[0],            0.0}};

    Vec3 v_dot = (Forces_Body/mass) - w_loc.cross(v_loc);
    Vec3 w_dot = inertia.inv()*(Moments_Body - w_loc.cross(inertia*w_loc));
    q_dot = ((omega*q_loc) + (q_loc*(1.0-q_loc.dot(q_loc))))*0.5;
    Vec3 P_dot = Body2NED(v_loc, q_loc);

    x_dot.data = {v_dot.data[0], v_dot.data[1], v_dot.data[2],
                  w_dot.data[0], w_dot.data[1], w_dot.data[2],
                  q_dot.data[0], q_dot.data[1], q_dot.data[2], q_dot.data[3],
                  P_dot.data[0], P_dot.data[1], P_dot.data[2]};

    return x_dot;
}