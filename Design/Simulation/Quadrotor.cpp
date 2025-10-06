#include "Quadrotor.h"

using namespace std;

Quadrotor::Quadrotor(Sim_Time Sim_dt, Sim_Time Sim_tf){
    // Sim timer initializations
    sim_dt = Sim_dt;
    sim_tf = Sim_tf;
    sim_t = {.Seconds = 0, .MicroSeconds = 0};
    Time_last_log = {.Seconds = 0, .MicroSeconds = 0};
    cal_start_time = {.Seconds = 5, .MicroSeconds = 0};
    // Initial conditions initialization
    q = Euler2Quat(Initial_Euler);
    // PIC32
    PIC.barometer.Initialize(200, 1, Bar_Mode_Bypass, 0);
    PIC.magnetometer.Initialize(100, 1);
    PIC.imu.Initialize(3330, 1660, 1, 2, 3);
    calibration_phase = 0;
    Moment_noise_gauss.Initialize(0.05, 0.0);
    Force_noise_guass.Initialize(0.05, 0.0);
}

void Quadrotor::Set_Monte_Carlo_Data(Monte_Carlo_Data MC_Data){
    mass = MC_Data.mass;
    length_f_b = MC_Data.length_f_b;
    length_l_r = MC_Data.length_l_r;
    inertia.data[0][0] =  MC_Data.interia_xx;
    inertia.data[1][1] =  MC_Data.inertia_yy;
    inertia.data[2][2] =  MC_Data.inertia_zz;
    inertia.data[0][1] = MC_Data.inertia_xy;
    inertia.data[1][0] = inertia.data[0][1];
    inertia.data[0][2] = MC_Data.inertia_xz;
    inertia.data[2][0] = inertia.data[0][2];
    inertia.data[1][2] = MC_Data.inertia_yz;
    inertia.data[2][1] = inertia.data[1][2];
    for (uint8_t i = 0; i < 4; i++){
        Motors[i].deadzone = MC_Data.Motor_deadzone[i];
        Motors[i].Motor_slope = MC_Data.Motor_slope;
        Motors[i].Motor_zero_offset = MC_Data.Motor_zero_offset;
        Motors[i].k_f = MC_Data.Propeller_force_constant;
        Motors[i].k_t = MC_Data.Propeller_torque_constant;
    }
    mu = MC_Data.Propeller_mu;
    Initial_Euler.data[0] = MC_Data.Initial_roll*D2R;
    Initial_Euler.data[1] = MC_Data.Initial_pitch*D2R;
    Initial_Euler.data[2] = MC_Data.Initial_yaw*D2R;

}

void Quadrotor::Calculate_errors(){
    if ((AVR128DB48.Flight_Controller_Status == Flying)||(AVR128DB48.Flight_Controller_Status == Landing)){
        for (uint8_t i = 0; i < 3; i++){
            Control_errors[i] += fabs(AVR128DB48.Desired_States.Euler[i] - Euler.data[i])*R2D*sim_dt.Time_fp();
            Control_errors[3+i] += fabs(AVR128DB48.Desired_States.Position_NED[i] - Position_NED.data[i])*sim_dt.Time_fp();
            Navigation_errors[i] += fabs(Euler.data[i] - AVR128DB48.mcu.Euler[i])*R2D*sim_dt.Time_fp();
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
        env.Update(Position_NED, q, v, a, w);

        Run_Sensors(env);

        Manage_FC_Status();

        PIC.Run(env, sim_t);

        Update_drone_forces_moments(env);

        Update_drone_states();

        if (log_flag) Log_data(env);

        if (error_flag) Calculate_errors();

        if (PIC.Flight_Controller_Status == Crashed_p32 || PIC.Successful_Landing) break;

        sim_t += sim_dt;
    }

    if (log_flag){
        log_sim.close();
        log_pic.close();
    }

    if (plot_flag) system("gnuplot plotter.plt");
}

void Quadrotor::Manage_FC_Status(){
    if (sim_t == cal_start_time){
        PIC.Flight_Controller_Status = User_Calibration_p32;
    }
    if (sim_t.Seconds - cal_start_time.Seconds >= 2){
        if (sim_t.Seconds - cal_start_time.Seconds <= 6){
            w.data[0] = 2.0;
            calibration_phase = 1;
        }
        else if (sim_t.Seconds - cal_start_time.Seconds <= 10){
            if (calibration_phase == 1){
                q.data[0] = 1; q.data[1] = 0; q.data[2] = 0; q.data[3] = 0;
                w.data[0] = -0.5;
            }
            w.data[1] = 3.0;
            calibration_phase = 2;
        }
        else if (sim_t.Seconds - cal_start_time.Seconds <= 14){
            if (calibration_phase == 2){
                q.data[0] = 1; q.data[1] = 0; q.data[2] = 0; q.data[3] = 0;
                w.data[1] = -0.75;
            }
            w.data[2] = -3.0;
            calibration_phase = 3;
        }
        else if (sim_t.Seconds - cal_start_time.Seconds <= 24){
            q = Euler2Quat(Initial_Euler);
            w.data[0] = 0.0;
            w.data[1] = 0.0;
            w.data[2] = 0.0;
            PIC.Flight_Controller_Status = System_Calibration_p32;
        }
        else if (sim_t.Seconds - cal_start_time.Seconds <= 26){
            PIC.Flight_Controller_Status = Ready_p32;
        }
        else if (sim_t.Seconds - cal_start_time.Seconds <= 28){
            PIC.Flight_Controller_Status = Flying_p32;
        }
    }
    // Assume that the initial drone orientation corresponds to the ground around it
    // If it pitches or rolls a certain distance past this initial orientation while near the ground,
    // it will be considered a crash
    if (PIC.Flight_Controller_Status == Flying_p32 || PIC.Flight_Controller_Status == Landing_p32){
        if (Position_NED.data[2] >= -0.15){
            if (fabs(Euler.data[0] - Initial_Euler.data[0]) >= (20*D2R) || fabs(Euler.data[1] - Initial_Euler.data[1]) >= (20*D2R)){
                PIC.Flight_Controller_Status = Crashed_p32;
            }
        }
    }
}

void Quadrotor::Run_Sensors(Environment &env){
    PIC.imu.Sample_Acc(env, sim_t);
    PIC.imu.Sample_Gyr(env, w, sim_t);
    PIC.magnetometer.Sample(env, sim_t);
    PIC.barometer.Sample(env, sim_t);
}

void Quadrotor::Log_data(Environment &env){
    const Sim_Time sim_init = {.Seconds = 0, .MicroSeconds = 0};
    const Sim_Time log_rate = {.Seconds = 0, .MicroSeconds = 10000};
    if (sim_t == sim_init){
        // Opens log file and deletes contents
        log_sim.open("Sim_log.txt", ios::out);
        LOG_DATA("Time", log_sim);
        LOG_DATA("P_n", log_sim);
        LOG_DATA("P_e", log_sim);
        LOG_DATA("P_h", log_sim);
        LOG_DATA("V_n", log_sim);
        LOG_DATA("V_e", log_sim);
        LOG_DATA("V_h", log_sim);
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
        log_pic.open("PIC_log.txt", ios::out);
        LOG_DATA("Time", log_pic);
        LOG_DATA("FC Status", log_pic);
        LOG_DATA("P_n", log_pic);
        LOG_DATA("P_e", log_pic);
        LOG_DATA("P_h", log_pic);
        LOG_DATA("V_n", log_pic);
        LOG_DATA("V_e", log_pic);
        LOG_DATA("V_h", log_pic);
        LOG_DATA("Filter_P_h", log_pic);
        LOG_DATA("Filter_V_h", log_pic);
        LOG_DATA("Pressure", log_pic);
        LOG_DATA("Roll", log_pic);
        LOG_DATA("Pitch", log_pic);
        LOG_DATA("Yaw", log_pic);
        LOG_DATA("w_x", log_pic);
        LOG_DATA("w_y", log_pic);
        LOG_DATA("w_z", log_pic);
        LOG_DATA("v_x", log_pic);
        LOG_DATA("v_y", log_pic);
        LOG_DATA("v_z", log_pic);
        log_pic << endl;
    }
    if (sim_t - Time_last_log >= log_rate && ((PIC.Flight_Controller_Status == Flying_p32) || (PIC.Flight_Controller_Status == Landing_p32))){
        Time_last_log = sim_t;
        w_deg_s = w*R2D;
        log_sim << setprecision(8);
        LOG_DATA(sim_t.Time_fp(), log_sim);
        LOG_DATA(Position_NED.data[0], log_sim);
        LOG_DATA(Position_NED.data[1], log_sim);
        LOG_DATA(-Position_NED.data[2], log_sim);
        LOG_DATA(Velocity_NED.data[0], log_sim);
        LOG_DATA(Velocity_NED.data[1], log_sim);
        LOG_DATA(-Velocity_NED.data[2], log_sim);
        LOG_DATA(env.pressure, log_sim);
        LOG_VEC3(Euler_deg, log_sim);
        LOG_VEC3(w_deg_s, log_sim);
        LOG_VEC3(v, log_sim);
        LOG_DATA(Motors[0].Get_motor_thrust(), log_sim);
        LOG_DATA(Motors[1].Get_motor_thrust(), log_sim);
        LOG_DATA(Motors[2].Get_motor_thrust(), log_sim);
        LOG_DATA(Motors[3].Get_motor_thrust(), log_sim);
        LOG_VEC3(Moments_Body, log_sim);
        log_sim << endl;
        log_pic << setprecision(8);
        LOG_DATA(sim_t.Time_fp(), log_pic);
        LOG_DATA(PIC.Flight_Controller_Status, log_pic);
        LOG_DATA(PIC.Output_States.Position_NED[0], log_pic);
        LOG_DATA(PIC.Output_States.Position_NED[1], log_pic);
        LOG_DATA(-PIC.Output_States.Position_NED[2], log_pic);
        LOG_DATA(PIC.Output_States.Velocity_NED[0], log_pic);
        LOG_DATA(PIC.Output_States.Velocity_NED[1], log_pic);
        LOG_DATA(-PIC.Output_States.Velocity_NED[2], log_pic);
        LOG_DATA(PIC.Output_States.Altitude_Filter_Data[0], log_pic);
        LOG_DATA(PIC.Output_States.Altitude_Filter_Data[1], log_pic);
        LOG_DATA(PIC.Output_States.pressure, log_pic);
        LOG_ARR3(PIC.Output_States.Euler_deg, log_pic);
        LOG_ARR3(PIC.Output_States.w_deg_s, log_pic);
        LOG_ARR3(PIC.Output_States.v, log_pic);
        log_pic << endl;
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
    // Assume that:
    // -> Back motor (0) produces negative pitching torque and positive yawing torque
    // -> Left motor (1) produces positive rolling torque and negative yawing torque
    // -> Right motor (2) produces negative rolling torque and negative yawing torque
    // -> Front motor (3) produces positive pitching torque and positive yawing torque

    for (uint8_t i = 0; i < 4; i++){
        Motors[i].Throttle = PIC.mapped_throttle_commands[i];
        Motors[i].Update_speed();
    }
    double motor_thrusts[4] = {Motors[0].Get_motor_thrust(), Motors[1].Get_motor_thrust(),
                          Motors[2].Get_motor_thrust(), Motors[3].Get_motor_thrust()};
    double motor_thrust_magnitude = motor_thrusts[0] + motor_thrusts[1] + motor_thrusts[2] + motor_thrusts[3];
    Vec3 motor_force_Body = {0.0, 0.0, -motor_thrust_magnitude};
    Vec3 motor_moment_Body = {
        length_l_r*(motor_thrusts[1] - motor_thrusts[2]),
        length_f_b*(motor_thrusts[3] - motor_thrusts[0]),
        Motors[0].Get_motor_torque() + Motors[3].Get_motor_torque() - Motors[1].Get_motor_torque() - Motors[2].Get_motor_torque()};

    // External, uncontrollable forces and moments
    Vec3 moment_Noise = {0.0, 0.0, 0.0};
    Vec3 force_Noise = {0.0, 0.0, 0.0};
    if (PIC.Flight_Controller_Status >= Flying_p32){
        moment_Noise = {Moment_noise_gauss.Get_val(), Moment_noise_gauss.Get_val(), Moment_noise_gauss.Get_val()};
        force_Noise = {Force_noise_guass.Get_val(), Force_noise_guass.Get_val(), Force_noise_guass.Get_val()};
    }

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
    // Drag force
    // Reference Quadrotors and Accelerometers
    // Proportional to the body translational velocity
    Vec3 drag_force_Body = {
        -v.data[0]*mu,
        -v.data[1]*mu,
        0.0
    };
    Forces_Body = g_force_Body + motor_force_Body + ground_forces_Body + drag_force_Body + force_Noise;
    Moments_Body = motor_moment_Body + moment_Noise;
}

void Quadrotor::Update_drone_states(){
    // Uses Runge Kutta 4th order integration to propogate momentum, NED to body quaternion, and NED position
    std::array<double, 13> x_2, k1, k2, k3, k4, temp, x_step;
    std::array<double, 13> x_1 = {v.data[0], v.data[1], v.data[2],
                                  w.data[0], w.data[1], w.data[2], 
                                  q.data[0], q.data[1], q.data[2], q.data[3],
                                  Position_NED.data[0], Position_NED.data[1], Position_NED.data[2]};
    double d_t = sim_dt.Time_fp();

    k1 = Differential_equation_momentum(x_1);
    for (uint_fast8_t i = 0; i < 13; i++){
        k1[i] *= d_t;
        temp[i] = x_1[i] + k1[i]*0.5;
    }

    k2 = Differential_equation_momentum(temp);
    for (uint_fast8_t i = 0; i < 13; i++){
        k2[i] *= d_t;
        temp[i] = x_1[i] + k2[i]*0.5;
    }

    k3 = Differential_equation_momentum(temp);
    for (uint_fast8_t i = 0; i < 13; i++){
        k3[i] *= d_t;
        temp[i] = x_1[i] + k3[i];
    }

    k4 = Differential_equation_momentum(temp);
    for (uint_fast8_t i = 0; i < 13; i++){
        k4[i] *= d_t;
        x_step[i] = (k1[i] + k2[i]*2.0 + k3[i]*2.0 + k4[i])*(1.0/6.0);
        x_2[i] = x_1[i] + x_step[i];
    }

    v = {x_2[0], x_2[1], x_2[2]};
    w = {x_2[3], x_2[4], x_2[5]};
    q.data = {x_2[6], x_2[7], x_2[8], x_2[9]};
    Position_NED = {x_2[10], x_2[11], x_2[12]};
    Velocity_NED = {x_step[10]/d_t, x_step[11]/d_t, x_step[12]/d_t};
    a = {x_step[0]/d_t, x_step[1]/d_t, x_step[2]/d_t};

    Euler = {atan2( 2.0*(q.data[0]*q.data[1] + q.data[2]*q.data[3]) , 1.0 - 2.0*(pow(q.data[1],2) + pow(q.data[2],2)) ),
             asin( 2.0*(q.data[0]*q.data[2] - q.data[1]*q.data[3]) ),
             atan2( 2.0*(q.data[0]*q.data[3] + q.data[1]*q.data[2]) , 1.0 - 2.0*(pow(q.data[2],2) + pow(q.data[3],2)) )};
    Euler_deg = Euler*R2D;
}

std::array<double, 13> Quadrotor::Differential_equation_momentum(std::array<double, 13> x_in){
    // Rigid body momentum equations in a rotating coordinate frame, 
    // feedback incorporated in quaternion equation to maintain magnitude 1
    // v_dot = F/m - w x v
    // w_dot = I^-1*(M - w x I*w)
    // q_dot = (omega*q)+(0.5*(1-dot(q,q))*q);
    // P_dot = Body2NED(v, q)
    Vec3 v_loc = {x_in[0], x_in[1], x_in[2]};
    Vec3 w_loc = {x_in[3], x_in[4], x_in[5]};
    Vec4 q_loc = {x_in[6], x_in[7], x_in[8], x_in[9]};

    Vec4 q_dot_uncomp = {-w_loc.data[0]*q_loc.data[1] - w_loc.data[1]*q_loc.data[2] - w_loc.data[2]*q_loc.data[3],
                          w_loc.data[0]*q_loc.data[0] + w_loc.data[2]*q_loc.data[2] - w_loc.data[1]*q_loc.data[3],
                          w_loc.data[1]*q_loc.data[0] - w_loc.data[2]*q_loc.data[1] + w_loc.data[0]*q_loc.data[3],
                          w_loc.data[2]*q_loc.data[0] + w_loc.data[1]*q_loc.data[1] - w_loc.data[0]*q_loc.data[2]};

    Vec3 v_dot = (Forces_Body/mass) - w_loc.cross(v_loc);
    Vec3 w_dot = inertia.inv()*(Moments_Body - w_loc.cross(inertia*w_loc));
    Vec4 q_dot = (q_dot_uncomp + (q_loc*(1.0-q_loc.dot(q_loc))))*0.5;
    Vec3 P_dot = Body2NED(v_loc, q_loc);

    std::array<double, 13> x_dot = {v_dot.data[0], v_dot.data[1], v_dot.data[2],
                                    w_dot.data[0], w_dot.data[1], w_dot.data[2],
                                    q_dot.data[0], q_dot.data[1], q_dot.data[2], q_dot.data[3],
                                    P_dot.data[0], P_dot.data[1], P_dot.data[2]};

    return x_dot;
}