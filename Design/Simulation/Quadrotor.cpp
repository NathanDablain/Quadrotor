#include "Quadrotor.h"
#include "External_Interface.h"
using namespace std;

Quadrotor::Quadrotor(Sim_Time Sim_dt, Sim_Time Sim_tf){
    // Sim timer initializations
    sim_dt = Sim_dt;
    sim_tf = Sim_tf;
    sim_t = {.Seconds = 0, .MicroSeconds = 0};
    Time_last_log = {.Seconds = 0, .MicroSeconds = 0};
    last_transmit_time = {.Seconds = 0, .MicroSeconds = 0};
    cal_start_time = {.Seconds = 5, .MicroSeconds = 0};
    Time_last_update_motors = {.Seconds = 0, .MicroSeconds = 0};
    Lora_ID_index = 0;
    // Initial conditions initialization
    q = Euler2Quat(Initial_Euler);
    // Reset microcontroller interface
    Reset_External_Interface();
    // PIC32
    Initialize_p32(log_flag);
    barometer.Initialize(e_bar_odr, false, e_bar_lpf_setting);
    magnetometer.Initialize(e_mag_odr, e_mag_lpf_setting);
    imu.Initialize(e_gyro_odr, e_accel_odr, e_gyro_lpf_setting, e_accel_lpf_setting);
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
        Motors[i].Motor_slope_l = MC_Data.Motor_slope_l;
        Motors[i].Motor_zero_offset_l = MC_Data.Motor_zero_offset_l;
        Motors[i].Motor_slope_h = MC_Data.Motor_slope_h;
        Motors[i].Motor_zero_offset_h = MC_Data.Motor_zero_offset_h;
        Motors[i].k_f = MC_Data.Propeller_force_constant;
        Motors[i].k_t = MC_Data.Propeller_torque_constant;
    }
    mu = MC_Data.Propeller_mu;
    Initial_Euler.data[0] = MC_Data.Initial_roll*D2R;
    Initial_Euler.data[1] = MC_Data.Initial_pitch*D2R;
    Initial_Euler.data[2] = MC_Data.Initial_yaw*D2R;

}

void Quadrotor::Run_sim(){
    Environment env(-97.06265, 32.79100, 0.0, sim_dt);

    while(sim_t <= sim_tf){
        e_Current_Time.seconds = sim_t.Seconds;
        e_Current_Time.tmr1_count = 25*sim_t.MicroSeconds;

        env.Update(Position_NED, q, v, a, w);

        Run_Sensors(env);

        Run_Ground_Controller();

        Execute_p32();

        Update_drone_forces_moments(env);

        Update_drone_states();

        if (log_flag) Log_data(env);

        if (inbound_Flight_Controller_Status == Crashed || Successful_Landing) break;

        sim_t += sim_dt;
    }

    if (log_flag){
        log_sim.close();
        log_pic.close();
    }

    if (plot_flag) system("gnuplot plotter.plt");
}

void Quadrotor::Run_Ground_Controller(){
    const Sim_Time transmit_rate = {.Seconds = 1, .MicroSeconds = 0};
    // Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C*CS
	const char ID[12][3] = {
		{'I','e',0},
		{'W','n',0},
		{'o','Y',0},
		{'u','o',0},
		{'l','u',0},
		{'d','r',0},
		{'H','D',0},
		{'a','a',0},
		{'v','d',0},
		{'e','d',0},
		{'B','y',0},
		{'e','.',0},
	};
    // Uplink the desired flight controller status and positions once per second
    if (sim_t - last_transmit_time > transmit_rate){
        last_transmit_time = sim_t;
        // Cycle message ID
        Lora_ID_index = (Lora_ID_index<11)?(Lora_ID_index+1):(0);
        char buffer[5][7];
        sprintf(buffer[0], "%06.2f", fabs(Lora_Desired_North));
        char north_south = (Lora_Desired_North >= 0)?('N'):('S');
        sprintf(buffer[1], "%06.2f", fabs(Lora_Desired_East));
        char east_west = (Lora_Desired_East >= 0)?('E'):('W');
        sprintf(buffer[2], "%06.2f", Lora_Desired_Altitude);
        sprintf(buffer[3], "%06.2f", fabs(Lora_Pressure_Altitude));
        sprintf(buffer[4], "%d", Lora_Desired_Status);
        // Build up link message
        char message[] = {'$', 'N', 'D', ID[Lora_ID_index][0], ID[Lora_ID_index][1],
		 buffer[0][0], buffer[0][1], buffer[0][2], buffer[0][3], buffer[0][4], buffer[0][5], north_south,
		 buffer[1][0], buffer[1][1], buffer[1][2], buffer[1][3], buffer[1][4], buffer[1][5], east_west,
		 buffer[2][0], buffer[2][1], buffer[2][2], buffer[2][3], buffer[2][4], buffer[2][5],
		 buffer[3][0], buffer[3][1], buffer[3][2], buffer[3][3], buffer[3][4], buffer[3][5], buffer[4][0],
		 '*', 0, 0, 0};
        // Build checksum
        char checksum_hex[3] = {0};
        uint8_t start_index = 1;
        uint8_t length = sizeof(message)-5;
        // checksum_hex must be a null terminated array of 3 characters
        int8_t checksum = message[start_index];
        for (uint8_t i=start_index+1; i<(length+start_index); i++){
            checksum ^= message[i];
        }
        uint8_t converted_length = snprintf(checksum_hex, 3, "%X", checksum);
        if (converted_length == 1){ // Won't add the 0 in automatically if the number is less than 8
            checksum_hex[1] = checksum_hex[0];
            checksum_hex[0] = '0';
        }
        message[sizeof(message)-2] = checksum_hex[1];
        message[sizeof(message)-3] = checksum_hex[0];
        // Copy into interface, don't include null terminator 
        memcpy(e_uplink_message, message, sizeof(message)-1);
        e_uplink_ready = true;
    }
    // When not transmitting, we are constantly checking for downlinks
    if (e_downlink_ready){
        e_downlink_ready = false;
        uint8_t data_available = sizeof(e_downlink_message);
        char buffer[sizeof(e_downlink_message)] = {0};
        memcpy(buffer, e_downlink_message, data_available);
        // Keeps track of index in buffer
        uint8_t i = 0;
        // Index in buffer where '$' is, signifies start of message
        int8_t start_index = -1;
        // Index in buffer where '*' is, signifies end of data section of message, beginning of checksum
        int8_t end_index = -1;
        // Populate with down link message checksum characters
        char Check_Sum[2] = {0};

        while(i != data_available){
            if (buffer[i] == '$'){
                start_index = i;
            }
            if ((start_index != -1)&&(buffer[i] == '*')){
                end_index = i;
                Check_Sum[0] = buffer[++i];
                Check_Sum[1] = buffer[++i];
                break;
            }
            i++;
        }

        if ((start_index == -1)||(end_index == -1)) return;
        // Compare checksum in message to calculated checksum
        char checksum_hex[3] = {0};
        // checksum_hex must be a null terminated array of 3 characters
        int8_t checksum = buffer[start_index+1];
        uint8_t length = 6;
        for (uint8_t i=start_index+2; i<(length+start_index+1); i++){
            checksum ^= buffer[i];
        }
        uint8_t converted_length = snprintf(checksum_hex, 3, "%X", checksum);
        if (converted_length == 1){ // Won't add the 0 in automatically if the number is less than 8
            checksum_hex[1] = checksum_hex[0];
            checksum_hex[0] = '0';
        }
        // If checksum passes, read downlink
        if ((checksum_hex[0] == Check_Sum[0])&&(checksum_hex[1] == Check_Sum[1])){
            char inbound_ID[3] = {buffer[3], buffer[4], 0};
            if (strcmp(inbound_ID, ID[Lora_ID_index]) == 0){
                inbound_Flight_Controller_Status = (FC_Status)buffer[5];
            }
        }
    }

    switch(inbound_Flight_Controller_Status){
        case Standby:
            if (sim_t == cal_start_time){
                Lora_Desired_Status = User_Calibration;
            }
            break;

        case User_Calibration:
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
                    Lora_Desired_Status = System_Calibration;
                }
            }
            break;
            
        case System_Calibration:
            if (sim_t.Seconds >= 26){
                Lora_Desired_Status = Ready;
            }
            break;
            
        case Ready:
            if (sim_t.Seconds >= 28){
                Lora_Desired_Altitude = 2.0;
                Lora_Desired_Status = Flying;
            }
            break;
            
        case Flying:
            if ((Position_NED.data[2] >= -0.15) && 
                (fabs(Euler.data[0] - Initial_Euler.data[0]) >= (20*D2R) ||
                 fabs(Euler.data[1] - Initial_Euler.data[1]) >= (20*D2R)))
            {
                inbound_Flight_Controller_Status = Crashed;
                Successful_Landing = false;
            }
            if ((-Position_NED.data[2] > 1.3*Lora_Desired_Altitude) || 
                (fabs(Position_NED.data[0]) > 15.0) ||
                (fabs(Position_NED.data[1]) > 15.0))
            {
                inbound_Flight_Controller_Status = Crashed;
                Successful_Landing = false;
            }

            if (sim_t.Seconds - cal_start_time.Seconds >= 60){
                if (fabs(-Position_NED.data[2] - Lora_Desired_Altitude) > Lora_Desired_Altitude*0.1){
                    inbound_Flight_Controller_Status = Crashed;
                    Successful_Landing = false; 
                }
                else {
                    Lora_Desired_Status = Landing;
                }
            }
            break;
            
        case Landing:
            if (Position_NED.data[2] >= -0.15){
                if ((fabs(Euler.data[0]) >= (10*D2R) ||
                    fabs(Euler.data[1]) >= (10*D2R)) ||
                    (Velocity_NED.data[2] > 1.0))
                {
                    inbound_Flight_Controller_Status = Crashed;
                    Successful_Landing = false;
                }
                else{
                    Successful_Landing = true;
                }
            }
            break;
            
        case Crashed:
            break;
            
    }

}

void Quadrotor::Run_Sensors(Environment &env){
    if (e_imu_settings_updated){
        imu.Initialize(e_gyro_odr, e_accel_odr, e_gyro_lpf_setting, e_accel_lpf_setting);
        e_imu_settings_updated = false;
    }
    if (e_mag_settings_updated){
        magnetometer.Initialize(e_mag_odr, e_mag_lpf_setting);
        e_mag_settings_updated = false;
    }
    if (e_bar_settings_updated){
        barometer.Initialize(e_bar_odr, e_bar_low_noise_setting, e_bar_lpf_setting);
        e_bar_settings_updated = false;
    }
    imu.Sample_Acc(env, sim_t);
    imu.Sample_Gyr(env, w, sim_t);
    magnetometer.Sample(env, sim_t);
    barometer.Sample(env, sim_t);
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
    if (sim_t - Time_last_log >= log_rate && ((inbound_Flight_Controller_Status == Flying) || (inbound_Flight_Controller_Status == Landing))){
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
        LOG_DATA(inbound_Flight_Controller_Status, log_pic);
        LOG_DATA(e_Output_States.Position_NED[0], log_pic);
        LOG_DATA(e_Output_States.Position_NED[1], log_pic);
        LOG_DATA(-e_Output_States.Position_NED[2], log_pic);
        LOG_DATA(e_Output_States.Velocity_NED[0], log_pic);
        LOG_DATA(e_Output_States.Velocity_NED[1], log_pic);
        LOG_DATA(-e_Output_States.Velocity_NED[2], log_pic);
        LOG_DATA(e_Output_States.pressure, log_pic);
        LOG_ARR3(e_Output_States.Euler_deg, log_pic);
        LOG_ARR3(e_Output_States.w_deg_s, log_pic);
        LOG_ARR3(e_Output_States.v, log_pic);
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
    // -> Front left (ESC 2, index 0) produces positive pitching torque, positive rolling torque, and negative yawing torque
    // -> Front right (ESC 4, index 1) produces positive pitching torque, negative rolling torque, and positive yawing torque
    // -> Back left (ESC 1, index 2) produces negative pitching torque, positive rolling torque, and positive yawing torque
    // -> Back right (ESC 3, index 3) produces negative pitching torque, negative rolling torque, and negative yawing torque
    const Sim_Time motor_update_rate = {.Seconds = 0, .MicroSeconds = 333};

    if (sim_t - Time_last_update_motors > motor_update_rate){
        Time_last_update_motors = sim_t;
        for (uint8_t i = 0; i < 4; i++){
            Motors[i].Throttle = e_throttle_commands[i];
            Motors[i].Update_speed();
        }
    }
    double motor_thrusts[4] = {Motors[0].Get_motor_thrust(), Motors[1].Get_motor_thrust(),
                          Motors[2].Get_motor_thrust(), Motors[3].Get_motor_thrust()};
    double motor_thrust_magnitude = motor_thrusts[0] + motor_thrusts[1] + motor_thrusts[2] + motor_thrusts[3];
    Vec3 motor_force_Body = {0.0, 0.0, -motor_thrust_magnitude};
    Vec3 motor_moment_Body = {
        length_l_r*(motor_thrusts[0] + motor_thrusts[1] - motor_thrusts[2] - motor_thrusts[3]),
        length_f_b*(motor_thrusts[1] + motor_thrusts[3] - motor_thrusts[0] - motor_thrusts[2]),
        Motors[0].Get_motor_torque() + Motors[3].Get_motor_torque() - Motors[1].Get_motor_torque() - Motors[2].Get_motor_torque()};

    // External, uncontrollable forces and moments
    Vec3 moment_Noise = {0.0, 0.0, 0.0};
    Vec3 force_Noise = {0.0, 0.0, 0.0};
    if (inbound_Flight_Controller_Status >= Flying){
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