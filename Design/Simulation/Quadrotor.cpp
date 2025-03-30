#include "Quadrotor.h"

using namespace std;

Quadrotor::Quadrotor(double Sim_dt, double Sim_tf){
    sim_dt = Sim_dt;
    sim_tf = Sim_tf;
    sim_t = 0.0;
    inertia.data[0][0] = Ixx;
    inertia.data[1][1] = Iyy;
    inertia.data[2][2] = Izz;
    q = {1.0, 0.0, 0.0, 0.0};
    Motors[0].deadzone = 300;
    Motors[1].deadzone = 125;
    Motors[2].deadzone = 130;
    Motors[3].deadzone = 120;

    Reference.Position_NED[2] = -5.0;
}

void Quadrotor::Run_sim(){
    Environment env(-97.06265, 32.79100, 0.0, sim_dt);

    Calibrate_sensors(env);
    MCU_Cal_Flag = true;

    while(sim_t < sim_tf){
        env.Update(Position_NED, q, v);

        Run_MCU(env);

        Update_drone_forces_moments(env.gravity, env.ground_stiffness, env.ground_damping);
        
        Update_drone_states();

        Log_data(env);

        sim_t += sim_dt;
    }
    cout << setprecision(8) << "  P0:" << Position_NED.data[0] << "   P1:" << Position_NED.data[1] << "  P2:" << Position_NED.data[2] << endl;
}

void Quadrotor::Log_data(Environment &env){
    if (sim_t == 0.0){
        ofstream log_sim("Sim_log.txt");
        LOG_SIM("Time");
        LOG_SIM("P_n");
        LOG_SIM("P_e");
        LOG_SIM("Height");
        LOG_SIM("Pressure");
        LOG_SIM("Roll");
        LOG_SIM("Pitch");
        LOG_SIM("Yaw");
        LOG_SIM("w_x");
        LOG_SIM("w_y");
        LOG_SIM("w_z");
        LOG_SIM("v_x");
        LOG_SIM("v_y");
        LOG_SIM("v_z");
        LOG_SIM("Back Motor Thrust");
        LOG_SIM("Left Motor Thrust");
        LOG_SIM("Right Motor Thrust");
        LOG_SIM("Front Motor Thrust");
        LOG_SIM(endl);
        log_sim.close();
        ofstream log_mcu("MCU_log.txt");
        LOG_MCU("Time");
        LOG_MCU("P_n");
        LOG_MCU("P_e");
        LOG_MCU("Height");
        LOG_MCU("Pressure");
        LOG_MCU("Roll");
        LOG_MCU("Pitch");
        LOG_MCU("Yaw");
        LOG_MCU("w_x");
        LOG_MCU("w_y");
        LOG_MCU("w_z");
        LOG_MCU("v_x");
        LOG_MCU("v_y");
        LOG_MCU("v_z");
        LOG_MCU("Roll Desired");
        LOG_MCU("Pitch Desired");
        LOG_MCU("Yaw Desired");
        LOG_MCU("w_x Desired");
        LOG_MCU("w_y Desired");
        LOG_MCU("w_z Desired");
        LOG_MCU(endl);
        log_mcu.close();
    }
    ofstream log_sim("Sim_log.txt", ios::app);
    log_sim << setprecision(8);
    LOG_SIM(sim_t);
    LOG_SIM(Position_NED.data[0]);
    LOG_SIM(Position_NED.data[1]);
    LOG_SIM(Position_NED.data[2]);
    LOG_SIM(env.pressure);
    LOG_SIM(Euler.data[0]);
    LOG_SIM(Euler.data[1]);
    LOG_SIM(Euler.data[2]);
    LOG_SIM(w.data[0]);
    LOG_SIM(w.data[1]);
    LOG_SIM(w.data[2]);
    LOG_SIM(v.data[0]);
    LOG_SIM(v.data[1]);
    LOG_SIM(v.data[2]);
    LOG_SIM(Motors[0].Get_motor_thrust());
    LOG_SIM(Motors[1].Get_motor_thrust());
    LOG_SIM(Motors[2].Get_motor_thrust());
    LOG_SIM(Motors[3].Get_motor_thrust());
    LOG_SIM(endl);
    log_sim.close();
    ofstream log_mcu("MCU_log.txt", ios::app);
    log_mcu << setprecision(8);
    LOG_MCU(sim_t);
    LOG_MCU(MCU.Position_NED[0]);
    LOG_MCU(MCU.Position_NED[1]);
    LOG_MCU(MCU.Position_NED[2]);
    LOG_MCU(MCU.pressure);
    LOG_MCU(MCU.Euler[0]);
    LOG_MCU(MCU.Euler[1]);
    LOG_MCU(MCU.Euler[2]);
    LOG_MCU(w.data[0]);
    LOG_MCU(w.data[1]);
    LOG_MCU(w.data[2]);
    LOG_MCU(v.data[0]);
    LOG_MCU(v.data[1]);
    LOG_MCU(v.data[2]);
    LOG_MCU(Reference.Euler[0]);
    LOG_MCU(Reference.Euler[1]);
    LOG_MCU(Reference.Euler[2]);
    LOG_MCU(Reference.w[0]);
    LOG_MCU(Reference.w[1]);
    LOG_MCU(Reference.w[2]);
    LOG_MCU(endl);
    log_mcu.close();
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
    for (uint8_t i = 0; i < 4; i++){
        Motors[i].Update_speed();
    }
    motor_thrusts = {Motors[0].Get_motor_thrust(), Motors[1].Get_motor_thrust(),
                     Motors[2].Get_motor_thrust(), Motors[3].Get_motor_thrust()};
    Vec3 motor_force_Body = {0.0, 0.0, -motor_thrusts.magnitude()};
    // Assume that:
    // -> Back motor (0) produces negative pitching torque and negative yawing torque
    // -> Left motor (1) produces positive rolling torque and positive yawing torque
    // -> Right motor (2) produces negative rolling torque and positive yawing torque
    // -> Front motor (3) produces positive pitching torque and negative yawing torque
    Vec3 motor_moment_Body = {
        length_l_r*(motor_thrusts.data[1] - motor_thrusts.data[2]),
        length_f_b*(motor_thrusts.data[3] - motor_thrusts.data[0]),
        length_l_r*(Motors[1].Get_motor_torque() + Motors[2].Get_motor_torque()) - length_f_b*(Motors[0].Get_motor_torque() + Motors[3].Get_motor_torque())};
    // Ground force
    // Model the ground as a lumped parameter model, it has some stiffness and some damping
    Vec3 ground_forces_NED;
    Vec3 ground_forces_Body;
    Vec3 Velocity_NED = Body2NED(v, q);
    if (Position_NED.data[2] >= 0){
        ground_forces_NED = {0.0, 0.0, -Position_NED.data[2]*ground_stiffness + -Velocity_NED.data[2]*ground_damping};
        ground_forces_Body = NED2Body(ground_forces_NED, q);
    }
    else{
        ground_forces_Body = {0.0, 0.0, 0.0};
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

Vec Quadrotor::Differential_equation_momentum(Vec x_in){
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

void Quadrotor::Run_MCU(Environment &env){
    // This method models the flight controller source code in src/flight_controller
    static double rtc_timelast, tcb0_timelast, tcb1_timelast, gps_timelast;
    static uint8_t seconds, LoRa_Read_Flag, Motor_Run_Flag, BAR_Read_Flag, Attitude_Observer_Run_Flag,
         MAG_Read_Flag, IMU_Read_Flag, GPS_Read_Flag, MRAC_Flag;
    static uint16_t motor_throttles[4] = {0};

    // Timing
    if (sim_t - rtc_timelast >= rtc_rate){
        ++seconds;
        ++LoRa_Read_Flag;
        rtc_timelast = sim_t;
    }

    if (sim_t - tcb0_timelast >= tcb0_rate){
        ++MRAC_Flag;
        ++Motor_Run_Flag;
        ++BAR_Read_Flag;
        ++Attitude_Observer_Run_Flag;
        ++MAG_Read_Flag;
        tcb0_timelast = sim_t;
    }
    
    if (sim_t - tcb1_timelast >= tcb1_rate){
        ++IMU_Read_Flag;
        tcb1_timelast = sim_t;
    }

    if (sim_t - gps_timelast >= gps_rate){
        ++GPS_Read_Flag;
        gps_timelast = sim_t;
    }

    // NAVIGATION //
/*
    if (GPS_Read_Flag){
        GPS_Read_Flag = 0;
        Read_GPS(MCU, env);
    }
*/
    if (BAR_Read_Flag >= 3){
        BAR_Read_Flag = 0;
        Read_Bar(MCU, env);
    }

    if (MAG_Read_Flag >= 2){
        MAG_Read_Flag = 0;
        Read_Mag(MCU, env);
    }

    if (IMU_Read_Flag){
        IMU_Read_Flag = 0;
        Read_IMU(MCU, env);
    }

    if (Attitude_Observer_Run_Flag >= 8){
        Attitude_Observer_Run_Flag = 0;
        Observer(MCU);
    }

    if (LoRa_Read_Flag){
        LoRa_Read_Flag = 0;
        Read_LoRa(Reference, env);
    }

    if (MCU_Cal_Flag){
        // GUIDANCE //
        static float desired_thrust;
        static float desired_moments[3];
        if (MRAC_Flag >= 5){
            MRAC_Flag = 0;
            desired_thrust = Height_MRAC(MCU, -Reference.Position_NED[2]);
            Attitude_LQR(MCU, Reference);
        }
        // CONTROL //
        if (Motor_Run_Flag >= 2){
            Motor_Run_Flag = 0;
            Angular_Rate_Control(MCU, Reference, desired_moments);
            Set_throttles(motor_throttles, desired_thrust, desired_moments);
            Run_Motors(motor_throttles);
        }
    }
}

void Quadrotor::Read_Bar(States &mcu, Environment &env){
	static uint32_t pressure_window[BAR_WINDOW_SIZE];
	static uint8_t window_counter;

    uint32_t pressure_LSB = env.Get_pressure();
	
	pressure_window[window_counter++] = pressure_LSB;
	
	if (window_counter >= BAR_WINDOW_SIZE){
		uint32_t pressure_oversampled = 0;
		for (uint8_t i=0;i<BAR_WINDOW_SIZE;i++){
			pressure_oversampled += pressure_window[i];
		}
		pressure_oversampled >>= 4;
        mcu.pressure = static_cast<float>(pressure_oversampled)*BAR_SENS;
		mcu.pressure_altitude = Height_Bar(pressure_oversampled);
		window_counter = 0;
	}
	
}

float Height_Bar(uint32_t pressure_LSB){
	const float c1 = BAR_TB/BAR_LB;
	const float c2 = (-BAR_R*BAR_LB)/(BAR_G*BAR_M);
	
	float pressure_Pa = ((float)pressure_LSB)*BAR_SENS;
	float height = c1*(pow(pressure_Pa/BAR_PB,c2)-1.0);
	return height;
}

void Quadrotor::Read_Mag(States &mcu, Environment &env){
    static int16_t 
        m_xyz_window[3][MAG_WINDOW_SIZE],
        m_max[3],
        m_min[3],
        hard_iron[3];
    static uint8_t window_counter;


    array<int16_t, 3> mag_field = env.Get_magnetic_field();

    for (uint8_t i=0;i<3;i++){
        m_xyz_window[i][window_counter] = mag_field[i];
    }
    window_counter++;
    if (window_counter >= MAG_WINDOW_SIZE){
        window_counter = 0;
        int32_t m_xyz_FIP[3] = {0};
        for (uint8_t i=0;i<3;i++){
            for (uint8_t j=0;j<MAG_WINDOW_SIZE;j++){
                m_xyz_FIP[i] += m_xyz_window[i][j];
            }
            m_xyz_FIP[i] >>= 4;
            if (m_xyz_FIP[i] > m_max[i]){
                m_max[i] = m_xyz_FIP[i];
                if (abs(m_max[i])<abs(m_min[i])){
                    hard_iron[i] = m_min[i]-m_max[i];
                }
                else{
                    hard_iron[i] = m_max[i]-m_min[i];
                }
                hard_iron[i] >>= 1;
            }
            else if (m_xyz_FIP[i] < m_min[i]){
                m_min[i] = m_xyz_FIP[i];
                if (abs(m_max[i])<abs(m_min[i])){
                    hard_iron[i] = m_min[i]-m_max[i];
                }
                else{
                    hard_iron[i] = m_max[i]-m_min[i];
                }
                hard_iron[i] >>= 1;
            }
        }
        if (sim_t > 2.0){
            mcu.m_vec[0] = ((float)(m_xyz_FIP[1] - hard_iron[1]))/(hard_iron[1]*2.0);
            mcu.m_vec[1] = -((float)(m_xyz_FIP[0]- hard_iron[0]))/(hard_iron[0]*2.0);
            mcu.m_vec[2] = ((float)(m_xyz_FIP[2] - hard_iron[2]))/(hard_iron[2]*2.0);
        }
    }

}

void Quadrotor::Read_IMU(States &mcu, Environment &env){
    static int16_t 
        a_xyz_window[3][IMU_WINDOW_SIZE],
        w_xyz_window[3][IMU_WINDOW_SIZE],
        w_bias[3];
    static uint8_t window_counter = 0;

    array<int16_t, 3> Data_g = env.Get_acceleration(q);
    array<int16_t, 3> Data_w = env.Get_angular_rate(w);

    for (uint8_t i=0;i<3;i++){
        a_xyz_window[i][window_counter] = Data_g[i];
        w_xyz_window[i][window_counter] = Data_w[i];
    }
    window_counter++;

    if (window_counter >= IMU_WINDOW_SIZE){
        window_counter = 0;
        int32_t a_xyz_FIR[3] = {0};
        int32_t w_xyz_FIR[3] = {0};
        for (uint8_t i=0;i<3;i++){
            for (uint8_t j=0;j<IMU_WINDOW_SIZE;j++){
                a_xyz_FIR[i] += a_xyz_window[i][j];
                w_xyz_FIR[i] += w_xyz_window[i][j];
            }
            a_xyz_FIR[i] >>= 3;
            w_xyz_FIR[i] >>= 3;
        }
        // Flip positive directions on Gyro x and z axis and Accelerometer y axis to align with Forward-Right-Down coordinate system (aligns with NED when not rotated)
        if (sim_t < 1.0){
            w_bias[0] = -w_xyz_FIR[0];
            w_bias[1] = w_xyz_FIR[1];
            w_bias[2] = -w_xyz_FIR[2];
        }
        int32_t w_diff[3] = {-w_xyz_FIR[0]-w_bias[0],
                              w_xyz_FIR[1]-w_bias[1], 
                             -w_xyz_FIR[2]-w_bias[2]};
        for (uint8_t i=0;i<3;i++){
            mcu.w[i] = ((float)w_diff[i])*GYRO_SENS*D2R;
        }
        mcu.g_vec[0] = ((float)a_xyz_FIR[0])*ACCEL_SENS;
        mcu.g_vec[1] = ((float)-a_xyz_FIR[1])*ACCEL_SENS;
        mcu.g_vec[2] = ((float)a_xyz_FIR[2])*ACCEL_SENS;
    }

}

void Quadrotor::Read_LoRa(States &reference, Environment &env){
    reference.Position_NED[2] = sim_t > 10.0 ? 0.0 : -5.0;
}

void Quadrotor::Calibrate_sensors(Environment &env){

    while(sim_t < cal_t){
        Position_NED = {0.0, 0.0, -0.5};

        env.Update(Position_NED, q, v);

        Run_MCU(env);

        Update_drone_forces_moments(env.gravity, env.ground_stiffness, env.ground_damping);
        
        Update_drone_states();

        Log_data(env);

        if (sim_t > gyro_cal_t + 0.5){
            w = {0.5, 0.0, PI};
        }

        sim_t += sim_dt;
    }

    Position_NED = {0.0, 0.0, 0.0};
    w = {0.0, 0.0, 0.0};
    v = {0.0, 0.0, 0.0};
    q = {1.0, 0.0, 0.0, 0.0};
}

void Quadrotor::Observer(States &mcu){
    const float 
		L = 0.05, // Observer gain, increasing the gain increases the trust on the model(gyro),
	// and decreasing it increases the trust on the measurement (accelerometer and magnetometer)
		dt = 0.04, // Time between integrations
		Gimbal_Lock_Check_Angle = 5.0*D2R;
	// Measure
	
	float phi_m = atan2f(mcu.g_vec[1], mcu.g_vec[2]);
	if (isnan(phi_m)){
		phi_m = mcu.Euler[0];
	}
	float theta_m = atan2f(-mcu.g_vec[0], sqrt(pow(mcu.g_vec[1],2) + pow(mcu.g_vec[2],2)));
	if (isnan(theta_m)){
		theta_m = mcu.Euler[1];
	}
	float mag_x_NED = cosf(mcu.Euler[1])*mcu.m_vec[0] + sinf(mcu.Euler[0])*sinf(mcu.Euler[1])*mcu.m_vec[1] + cosf(mcu.Euler[0])*sinf(mcu.Euler[1])*mcu.m_vec[2];
	float mag_y_NED = cosf(mcu.Euler[0])*mcu.m_vec[1] - sinf(mcu.Euler[0])*mcu.m_vec[2];
	float psi_m = -atan2f(mag_y_NED, mag_x_NED);
	if (isnan(psi_m)){
		psi_m = mcu.Euler[2];
	}
	if (psi_m <= 0){
		psi_m += 2.0*PI;
	}
	
	// Predict
	float phi_hat = mcu.Euler[0];
	float theta_hat = mcu.Euler[1];
	float psi_hat = mcu.Euler[2];
	if (abs(abs(mcu.Euler[1]) - PI_2) > Gimbal_Lock_Check_Angle){
		phi_hat += (mcu.w[0] + sinf(mcu.Euler[0])*tanf(mcu.Euler[1])*mcu.w[1] + cosf(mcu.Euler[0])*tanf(mcu.Euler[1])*mcu.w[2])*dt;
		theta_hat += (cosf(mcu.Euler[0])*mcu.w[1] - sinf(mcu.Euler[0])*mcu.w[2])*dt;
		psi_hat += ((sinf(mcu.Euler[0])/cosf(mcu.Euler[1]))*mcu.w[1] + (cosf(mcu.Euler[0])/cosf(mcu.Euler[1]))*mcu.w[2])*dt;
	}
	// Update
	mcu.Euler[0] = phi_hat + L*(phi_m-phi_hat);
	mcu.Euler[1] = theta_hat + L*(theta_m-theta_hat);
	// Prevent yaw angle discontinuity at 2pi - 0 to cause the filter to slowly cycle between them
	mcu.Euler[2] = (abs(psi_m-psi_hat)>PI)?(psi_m):(psi_hat + L*(psi_m-psi_hat));
    // cout << sim_t << "   " << mcu.Euler[0] << "   " << mcu.Euler[1] << "   " << mcu.Euler[2] << endl;
}

void Quadrotor::Run_Motors(uint16_t motor_throttles[4]){
    static uint16_t motor_lookup[1001] = {0};
	// Build the lookup table if it hasn't been built yet
	if (!(motor_lookup[0])){ 
		for (uint16_t i=0;i<1001;i++){
			motor_lookup[i] = 3*i + 3000;
		}
	}
	uint16_t mapped_throttle_commands[4] = {0};
	// Map commands, saturate if out of bounds
	for (uint8_t i=0;i<4;i++){
		motor_throttles[i] = (motor_throttles[i]>1000)?1000:motor_throttles[i];
		mapped_throttle_commands[i] = motor_lookup[motor_throttles[i]];
        Motors[i].Throttle = mapped_throttle_commands[i];
	}
}