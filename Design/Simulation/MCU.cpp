#include "MCU.h"

void MCU::Run(Environment &env, Sim_Time sim_t){
    // This method models the flight controller source code in src/flight_controller
    Run_Timers(sim_t);
    // Bar samples at 75 Hz
    if (BAR_Read_Flag >= 2) Read_Bar();
    // Mag samples at 50 Hz
    if (MAG_Read_Flag) Read_Mag();
    // Gyro samples at 416 Hz
    if (Gyro_Read_Flag) Read_Gyro();
    // Accel samples at 52 Hz
    if (Accel_Read_Flag) Read_Accel();

    // Observer update -> 50Hz
    if (Attitude_Observer_Update_Flag >= 4) Observer_Update();
        
    // Observer predict -> 400Hz
    if (Attitude_Observer_Predict_Flag) Observer_Predict();

    if (LoRa_Read_Flag) Read_LoRa(env);

    if (Flight_Controller_Status == Standby){
        if (reset == 0){
            memset(&cal_data, 0, sizeof(cal_data));
            mcu.Latitude = -1;
            mcu.Longitude = -1;
            memset(motor_throttles, 0, sizeof(motor_throttles));
            reset++;
        }
    }
    else if (Flight_Controller_Status == Calibrating){ 
        reset = 0;
        if (cal_data.bar_cal_status == 0) Calibrate_Bar();
        
        if (cal_data.mag_cal_status == 0) Calibrate_Mag();
        
        if (cal_data.imu_cal_status == 0) Calibrate_Gyro();

        if (cal_data.bar_cal_status && cal_data.mag_cal_status && cal_data.imu_cal_status){
            Flight_Controller_Status = Ready;
            ready_time = sim_t;
        } 

    }
    else if (Flight_Controller_Status == Ready){
        if (sim_t.Seconds - ready_time.Seconds > 2){
            Desired_Position_NED[2] = -5;
            Desired_Euler[0] = 0;
            Desired_Euler[1] = 0;
            Desired_Euler[2] = 0;
            Flight_Controller_Status = Flying;
        }
    }
    else {
    //------------Guidance and Control functions-------------//
        if (Flight_Controller_Status == Flying){
            if (sim_t.Seconds - ready_time.Seconds > 15){
                Desired_Position_NED[2] = -5.5;
                //Desired_Euler[1] = 1*D2R; //15*D2R;
            }
    
            if (sim_t.Seconds - ready_time.Seconds > 25){
                Desired_Position_NED[2] = -4.5;
            }
    
            if (sim_t.Seconds - ready_time.Seconds > 40){
                Flight_Controller_Status = Landing;
            }
        }
        else if (Flight_Controller_Status == Landing){
            Desired_Position_NED[2] = 0.25;
            if (mcu.Position_NED[2] >= 0){
                Flight_Controller_Status = Standby;
                Desired_Thrust = 0.0;
                memset(Desired_Moments, 0, sizeof(Desired_Moments));
            }
        }
        
        if (Guidance_Flag){
            Guidance_Flag = 0;
            Run_Guidance();
        }
        // Altitude Controller Runs at 100 Hz, updates desired thrust
        if (Altitude_Control_Flag>=2){
            Altitude_Control_Flag = 0;
            Desired_Thrust = Height_LQR(-mcu.Position_NED[2], -Reference.Position_NED[2]);
        }

        // PIDs and ESCs run at 400 Hz, updates desired motor speeds
        if (Motor_Run_Flag){
            Angular_Rate_Control(mcu, Reference, Desired_Moments);
            Set_throttles(motor_throttles, Desired_Thrust, Desired_Moments);
        }
    }
    if (Motor_Run_Flag){
        Motor_Run_Flag = 0;
        Run_Motors(motor_throttles);
    }
}

void MCU::Run_Timers(Sim_Time sim_t){
    // 1Hz timer
    if (sim_t - rtc_timelast >= rtc_rate){
        ++seconds;
        ++LoRa_Read_Flag;
        rtc_timelast = sim_t;
    }
    // 200Hz timer
    if (sim_t - tcb0_timelast >= tcb0_rate){
        ++Altitude_Control_Flag;
        ++LQR_Flag;
        ++BAR_Read_Flag;
        ++Attitude_Observer_Update_Flag;
        ++MAG_Read_Flag;
        ++Accel_Read_Flag;
        ++Guidance_Flag;
        tcb0_timelast = sim_t;
    }
    // 416Hz timer
    if (sim_t - tcb1_timelast >= tcb1_rate){
        ++Motor_Run_Flag;
        ++Attitude_Observer_Predict_Flag;
        tcb1_timelast = sim_t;
    }
    // 1666Hz timer
    if (sim_t - tcb2_timelast >= tcb2_rate){
        ++Gyro_Read_Flag;
        tcb2_timelast = sim_t;
    }
}

void MCU::Calibrate_Bar(){
	if (abs(mcu.pressure_altitude-up_link.Base_altitude) < BAR_MAX_CAL_DIFF){
		cal_data.altitude_bias = mcu.pressure_altitude - up_link.Base_altitude;
		cal_data.bar_cal_status = 1;
	}
}

void MCU::Read_Bar(){
    BAR_Read_Flag = 0;
    if (barometer.drdy_flag == false) return;
    barometer.drdy_flag = false;
    // MRAC_Flag = 1;
    // if (barometer.FIFO_index < BAR_WINDOW_SIZE) return;
	
	// uint32_t Data[BAR_WINDOW_SIZE];
    // barometer.Read_FIFO(Data);
	// uint32_t pressure_oversampled = 0;
	// for (uint8_t i = 0; i < BAR_WINDOW_SIZE; i++){
	// 	pressure_oversampled += Data[i];
	// }
	// pressure_oversampled >>= 4;
    uint32_t pressure_oversampled = barometer.Pressure_Out_LSB;
    mcu.pressure = static_cast<double>(pressure_oversampled)*BAR_SENS;
	mcu.pressure_altitude = Height_Bar(pressure_oversampled);
	mcu.Position_NED[2] = -(mcu.pressure_altitude - (cal_data.altitude_bias + up_link.Base_altitude));
}

float Height_Bar(uint32_t pressure_LSB){
	const float c1 = BAR_TB/BAR_LB;
	const float c2 = (-BAR_R*BAR_LB)/(BAR_G*BAR_M);
	
	float pressure_Pa = ((float)pressure_LSB)*BAR_SENS;
	float height = c1*(pow(pressure_Pa/BAR_PB,c2)-1.0);
	return height;
}

void MCU::Calibrate_Mag(){
    static uint32_t initial_time;
	
	for (uint8_t i = 0; i < 3; i++){
		uint8_t calculate_hard_iron = 0;
		if (mcu.m_xyz_LSB[i] > cal_data.m_max[i]){
			cal_data.m_max[i] = mcu.m_xyz_LSB[i];
			calculate_hard_iron = 1;
		}
		else if (mcu.m_xyz_LSB[i] < cal_data.m_min[i]){
			cal_data.m_min[i] = mcu.m_xyz_LSB[i];
			calculate_hard_iron = 1;
		}
		if (calculate_hard_iron){
			initial_time = seconds;
			if (abs(cal_data.m_max[i])<abs(cal_data.m_min[i])){
				cal_data.hard_iron[i] = cal_data.m_min[i]-cal_data.m_max[i];
			}
			else{
				cal_data.hard_iron[i] = cal_data.m_max[i]-cal_data.m_min[i];
			}
			cal_data.hard_iron[i] >>= 1;
		}
	}

	if ((seconds - initial_time) >= MAG_CAL_TIMEOUT){
        for (uint8_t i = 0 ; i < 3; i++){
            if (cal_data.hard_iron[i] == 0) cal_data.hard_iron[i] = 1;
        }
		cal_data.mag_cal_status = 1;
	}
}

void MCU::Read_Mag(){	
	MAG_Read_Flag = 0;
    if (magnetometer.drdy_flag == false) return;
	magnetometer.drdy_flag = false;
	
	mcu.m_xyz_LSB[0] = magnetometer.magnetic_field_LSB[0];
    mcu.m_xyz_LSB[1] = magnetometer.magnetic_field_LSB[1]; 
	mcu.m_xyz_LSB[2] = magnetometer.magnetic_field_LSB[2]; 

	// mcu.m_vec[0] = ((float)(mcu.m_xyz_LSB[1] - cal_data.hard_iron[1]))/((float)cal_data.hard_iron[1]*2.0);
	// mcu.m_vec[1] = -((float)(mcu.m_xyz_LSB[0] - cal_data.hard_iron[0]))/((float)cal_data.hard_iron[0]*2.0);
	// mcu.m_vec[2] = ((float)(mcu.m_xyz_LSB[2] - cal_data.hard_iron[2]))/((float)cal_data.hard_iron[2]*2.0);
    mcu.m_vec[0] = (float)mcu.m_xyz_LSB[1];
    mcu.m_vec[1] = -(float)mcu.m_xyz_LSB[0];
	mcu.m_vec[2] = (float)mcu.m_xyz_LSB[2];
}

void MCU::Calibrate_Gyro(){
	// Flip positive directions on Gyro x and z axis to align with Forward-Right-Down coordinate system (aligns with NED when not rotated)
	unsigned long w_magnitude = abs(mcu.w[0]) + abs(mcu.w[1]) + abs(mcu.w[2]);
	
	if (w_magnitude <= W_CAL_LIMIT){
		cal_data.w_bias[0] = -mcu.w[0];
		cal_data.w_bias[1] = mcu.w[1];
		cal_data.w_bias[2] = -mcu.w[2];
		cal_data.imu_cal_status = 1;
	}
}

void MCU::Read_Gyro(){	
    Gyro_Read_Flag = 0;
    if (imu.gyro_drdy_flag == false) return;
    imu.gyro_drdy_flag = true;
	
	mcu.w[0] = -imu.angular_rate_LSB[0] - cal_data.w_bias[0];
	mcu.w[1] = imu.angular_rate_LSB[1] - cal_data.w_bias[1];
	mcu.w[2] = -imu.angular_rate_LSB[2] - cal_data.w_bias[2];
}

void MCU::Read_Accel(){
    Accel_Read_Flag = 0;
    if (imu.FIFO_index < ACCEL_WINDOW_SIZE) return;
	
    array<int16_t, 3> Data[ACCEL_WINDOW_SIZE];
    imu.Read_FIFO(Data);
	
	int32_t a_xyz_oversampled[3] = {0};
	for (uint8_t i=0; i<ACCEL_WINDOW_SIZE; i++){
		a_xyz_oversampled[0] += Data[i][0];
		a_xyz_oversampled[1] += Data[i][1];
		a_xyz_oversampled[2] += Data[i][2];
	}
	a_xyz_oversampled[0] >>= 3;
	a_xyz_oversampled[1] >>= 3;
	a_xyz_oversampled[2] >>= 3;

	// Flip positive directions on Accelerometer y axis to align with Forward-Right-Down coordinate system (aligns with NED when not rotated)
	mcu.g_vec[0] = a_xyz_oversampled[0];
	mcu.g_vec[1] = -a_xyz_oversampled[1];
	mcu.g_vec[2] = a_xyz_oversampled[2];
}

void MCU::Read_LoRa(Environment &env){
    LoRa_Read_Flag = 0;
    // reference.Position_NED[2] = sim_t > 10.0 ? 0.0 : -5.0;
    up_link.Base_altitude = 52.75;
}

void MCU::Observer_Predict(){
    Attitude_Observer_Predict_Flag = 0;
    // Predict
	if (abs(abs(mcu.Euler[1]) - PI_2) > OBSERVER_GIMBAL_LOCK_CHECK){
		mcu.Euler[0] += (mcu.w[0] + sinf(mcu.Euler[0])*tanf(mcu.Euler[1])*mcu.w[1] + cosf(mcu.Euler[0])*tanf(mcu.Euler[1])*mcu.w[2])*OBSERVER_DT*GYRO_SENS*D2R;
		mcu.Euler[1] += (cosf(mcu.Euler[0])*mcu.w[1] - sinf(mcu.Euler[0])*mcu.w[2])*OBSERVER_DT*GYRO_SENS*D2R;
		mcu.Euler[2] += ((sinf(mcu.Euler[0])/cosf(mcu.Euler[1]))*mcu.w[1] + (cosf(mcu.Euler[0])/cosf(mcu.Euler[1]))*mcu.w[2])*OBSERVER_DT*GYRO_SENS*D2R;
	}
}

void MCU::Observer_Update(){
    Attitude_Observer_Update_Flag = 0;
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
    // if (psi_m <= 0){
    //     psi_m += 2.0*PI;
    // }

    // Update
    mcu.Euler[0] = mcu.Euler[0] + OBSERVER_GAIN*(phi_m - mcu.Euler[0]);
    mcu.Euler[1] = mcu.Euler[1] + OBSERVER_GAIN*(theta_m - mcu.Euler[1]);
    // Prevent yaw angle discontinuity at 2pi - 0 to cause the filter to slowly cycle between them
    mcu.Euler[2] = (abs(psi_m - mcu.Euler[2])>PI)?(psi_m):(mcu.Euler[2] + OBSERVER_GAIN*(psi_m - mcu.Euler[2]));

}

void MCU::Run_Motors(uint16_t motor_throttles[4]){
    static uint16_t motor_lookup[1001] = {0};
	// Build the lookup table if it hasn't been built yet
	if (!(motor_lookup[0])){ 
		for (uint16_t i=0;i<1001;i++){
			motor_lookup[i] = 3*i + 3000;
		}
	}
	// Map commands, saturate if out of bounds
	for (uint8_t i=0;i<4;i++){
		motor_throttles[i] = (motor_throttles[i]>1000)?1000:motor_throttles[i];
		mapped_throttle_commands[i] = motor_lookup[motor_throttles[i]];
	}
}

void MCU::Run_Guidance(){
    // static float local_Desired_Position_NED[3];
    // static float local_Desired_Euler[3];
    const float c1 = 0.99;
    const float c2 = 1.0 - c1;
    for (uint8_t i = 0; i < 3; i++){
        Reference.Euler[i] = Reference.Euler[i]*c1 + Desired_Euler[i]*c2;
        Reference.Position_NED[i] = Reference.Position_NED[i]*c1 + Desired_Position_NED[i]*c2;
    }
}