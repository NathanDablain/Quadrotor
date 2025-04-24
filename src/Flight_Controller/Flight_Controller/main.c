#include "Flight_Controller.h"

volatile unsigned long g_seconds = 0;
	
unsigned char Setup(){
	if (RSTCTRL_RSTFR & RSTCTRL_PORF_bm){Delay(100000);} // Necessary to stabilize IC's on a cold start
	unsigned char Setup_Bitmask = 0;
	// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
	//					SSD	   LoRa	    MAG		IMU		BAR		GPS
	
	_PROTECTED_WRITE (CLKCTRL_OSCHFCTRLA, (CLKCTRL_FRQSEL_24M_gc|CLKCTRL_AUTOTUNE_bm)); // Sets CPU clock to 24 MHz
	while(!(CLKCTRL_MCLKSTATUS & CLKCTRL_OSCHFS_bm)); // Wait for clock to stabilize
	unsigned char GPS_setup_status = Setup_GPS();
	Setup_SPI();
	Setup_TWI();
	unsigned char LoRa_setup_status = Setup_LoRa();
	unsigned char MAG_setup_status = Setup_Mag();
	unsigned char IMU_setup_status = Setup_IMU();
	unsigned char BAR_setup_status = Setup_Bar();
	unsigned char SSD_setup_status = Setup_SSD();
	Setup_Bitmask |= (GPS_setup_status<<NAV_GPS_bp) | (BAR_setup_status<<NAV_BAR_bp) | (IMU_setup_status<<NAV_IMU_bp) | (MAG_setup_status<<NAV_MAG_bp)
	| (LoRa_setup_status<<NAV_LORA_bp) | (SSD_setup_status<<SU_SSD_bp);
	Setup_Timers();
	sei();
	return Setup_Bitmask;
}

void Setup_Timers(){
	//-Setup Real Time Clock for keeping track of total run time-//
	RTC_CTRLA |= RTC_CORREN_bm | RTC_RTCEN_bm;
	RTC_INTCTRL |= RTC_CMP_bm;
	RTC_CMP = 32768;
	//----------------------------------------------------------//
	//--------Setup Timer/Counter A0 for output compare---------//
	// Is triggered every 10 ms, is used by:
	//  -> Motors
	TCA0_SINGLE_CTRLA |= TCA_SINGLE_CLKSEL_DIV8_gc;
	TCA0_SINGLE_INTCTRL |= TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm;
	TCA1_SINGLE_INTCTRL |= TCA_SINGLE_CMP0_bm;
	//---------------------------------------------------------//
	//-------Setup Timer/Counter B0 for output compare---------//
	// Generates an interrupt every 5 ms, is used by:
	//  -> Motors running at 100 Hz
	//	-> Magnetometer running at 100 Hz
	//  -> Barometer running at 75 Hz
	//	-> Attitude observer running at 25 Hz
	//	-> Print statements, variable frequency
	TCB0_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV2_gc; // Enables timer, uses main clock with a prescaler of two
	TCB0_INTCTRL |= TCB_CAPT_bm; // Enables interrupt on capture
	TCB0_CCMP = 60000; // Value at which timer generates interrupt and resets
	//--------------------------------------------------------//
	//-------Setup Timer/Counter B1 for output compare--------//
	// Generates an interrupt every 2.4 ms, is used by:
	//	-> Accel and Observer running at 416 Hz
	TCB1_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV1_gc;
	TCB1_INTCTRL |= TCB_CAPT_bm; // Generate interrupts twice as fast as needed, trigger gyro on one and predictor on other
	TCB1_CCMP = 57600;
	//-------------------------------------------------------//
	//-------Setup Timer/Counter B2 for output compare--------//
	// Generates an interrupt every 0.6002 ms, is used by:
	//	-> Gyro running at 1666 Hz
	TCB2_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV1_gc;
	TCB2_INTCTRL |= TCB_CAPT_bm;
	TCB2_CCMP = 14405;
	//-------------------------------------------------------//
}

int main(){
	unsigned char Setup_Bitmask = Setup();
	// If the sensors we need for navigation initialized successfully, enter main loop
	if ((Setup_Bitmask & NAV_SENSORS_bm) == NAV_SENSORS_bm){
		// Initialize data structures
		// Drone-> tracks the current drone states
		States Drone;
		// Reference-> tracks the desired drone states used by guidance and control functions
		States Reference;
		// up_link-> contains the last information sent to the drone via LoRa uplink
		Uplink up_link;
		// down_link-> contains the flight controller calibration status and how well the drone is tracking references
		Downlink down_link = {0};
		// Flight_Controller_Status-> Controls the mode of operation the drone is in, changed by uplinks from the ground controller
		FC_Status Flight_Controller_Status = Standby;
		// Holds sensor calibration data 
		Calibration_Data cal_data = {0};
		const Calibration_Data reset_cal_data = {0};
		// Motor_Throttles-> Values from 0-1000 with 1000 being max throttle, motor order is: back, left, right, front
		unsigned int Motor_Throttles[4];
		// Desired_Thrust-> Controlled by a Model Reference Adaptive Controller (MRAC), in units of N
		float Desired_Thrust;
		// Desired_Moments-> Controlled by 3 PIDs, moment order is: body x, body y, body z, in units of N-m
		float Desired_Moments[3];
		unsigned int throughput_counter[7] = {0};
		volatile unsigned long unused_time_counter = 0;
		unsigned int seconds_last = g_seconds;
		unsigned char reset = 0;
		while(1){
			// READ STATUS REGISTERS OF SENSORS TO DETERMINE WHEN TO SAMPLE, READ THEM AT TWICE THE OUTPUT RATE
			//--------------Common code--------------//
			// LoRa
			if (g_LoRa_Check_Flag>=2){
				if (g_LoRa_Send_Flag) Send_Downlink(&down_link);
				 Receive_Uplink(&up_link, &down_link, &Flight_Controller_Status);
				 ++throughput_counter[0];
			}
			// Printing
			if (g_Print_Flag >= 20 &&(Setup_Bitmask & (1<<SU_SSD_bp))){
				g_Print_Flag = 0;
				char p_buffer[20];
				unsigned char length_to_print = snprintf(p_buffer, sizeof(p_buffer), "%3.1f, %3.1f, %3.1f", Drone.Euler[0], Drone.Euler[1], Drone.Euler[2]);
				Print_Page(3, p_buffer, length_to_print);
				 //Print_Output(&Drone, &cal_data, &up_link);
			}
			if ((g_seconds - seconds_last) >= 1){
				seconds_last = g_seconds;
				char buffer[3][20] = {0};
				unsigned char length_to_print = snprintf(buffer[0], sizeof(buffer[0]), "B:%d,M:%d,A:%d", throughput_counter[1], throughput_counter[2], throughput_counter[3]);
				Print_Page(0, buffer[0], length_to_print);
				length_to_print = snprintf(buffer[1], sizeof(buffer[1]), "G:%d,U:%d,P:%d",throughput_counter[4], throughput_counter[5], throughput_counter[6]);
				Print_Page(1, buffer[1], length_to_print);
				length_to_print = snprintf(buffer[2], sizeof(buffer[2]), "%ld", unused_time_counter);
				Print_Page(2, buffer[2], length_to_print);
				for (unsigned char i = 0; i < 7; i++){throughput_counter[i] = 0;}
				unused_time_counter = 0;
			}

			//// GPS
			//if (g_GPS_Read_Flag) Read_GPS(&Drone, &cal_data);
//
			// Barometer -> check 10Hz, samples at 4.6Hz
			if (g_BAR_Read_Flag >= 20){
				unsigned char bar_status = Read_Bar(&Drone, &cal_data, up_link.Base_altitude);
				if (bar_status) throughput_counter[1]++;
			}

			// Magnetometer -> check at 200 Hz, samples at 50Hz
			if (g_MAG_Read_Flag){
				 unsigned char mag_status = Read_Mag(&Drone, &cal_data);
				 if (mag_status) throughput_counter[2]++;
			}
			
			// Accelerometer -> check at 200 Hz, samples at 52Hz
			if (g_Accel_Read_Flag){ 
				unsigned char accel_status = Read_Accel(&Drone);
				if (accel_status) throughput_counter[3]++;
			}

			// Gyro -> Check at 1666Hz, samples at 416Hz
			if (g_Gyro_Read_Flag){
				 unsigned char gyro_status = Read_Gyro(&Drone, &cal_data); 
				 if (gyro_status) throughput_counter[4]++;}

			// Observer update -> 50Hz
			if (g_Attitude_Observer_Update_Flag >= 4){ Observer_Update(&Drone); throughput_counter[5]++;}
				
			// Observer predict -> 400Hz
			if (g_Attitude_Observer_Predict_Flag){ Observer_Predict(&Drone); throughput_counter[6]++;}

			//unused_time_counter++;
			//--------------Status code-------------//
			if (Flight_Controller_Status == Standby){
				// Initialize or reset calibration data
				if (reset == 0){
					cal_data = reset_cal_data;
					Drone.Latitude = -1;
					Drone.Longitude = -1;
					reset++;
				}
			}
			else if (Flight_Controller_Status == Calibrating){
				reset = 0;
				//if (cal_data.gps_cal_status == 0) Calibrate_GPS(&Drone, &cal_data);
				
				if (cal_data.bar_cal_status == 0) Calibrate_Bar(&Drone, &cal_data, up_link.Base_altitude);
				
				if (cal_data.mag_cal_status == 0) Calibrate_Mag(&Drone, &cal_data);
				
				if (cal_data.imu_cal_status == 0) Calibrate_IMU(&Drone, &cal_data);

				if (cal_data.bar_cal_status && cal_data.mag_cal_status && cal_data.imu_cal_status) Flight_Controller_Status = Ready;
	
			}
			else if (Flight_Controller_Status == Ready){
				
			}
			else if (Flight_Controller_Status == Flying){
				//------------Guidance and Control functions-------------//
				if (g_MRAC_Flag >= 50){
					g_MRAC_Flag = 0;
					Desired_Thrust = 4.32; //Height_MRAC(&Drone, -Reference.Position_NED[2]);
				}
				
				if (g_LQR_Flag >= 8) Attitude_LQR(&Drone, &Reference);
				
				if (g_Motor_Run_Flag >= 2){
					g_Motor_Run_Flag = 0;
					Angular_Rate_Control(&Drone, &Reference, Desired_Moments);
					Set_throttles(Motor_Throttles, Desired_Thrust, Desired_Moments);
					Run_Motors(Motor_Throttles);
				}
			}
			else if (Flight_Controller_Status == Landing){
				//------------Guidance and Control functions-------------//
				if (g_LQR_Flag >= 8) Attitude_LQR(&Drone, &Reference);
								
				if (g_Motor_Run_Flag >= 2){
					g_Motor_Run_Flag = 0;
					Angular_Rate_Control(&Drone, &Reference, Desired_Moments);
					Set_throttles(Motor_Throttles, Desired_Thrust, Desired_Moments);
					Run_Motors(Motor_Throttles);
				}
			}
		}

	}
	
	return 0;
}

ISR(RTC_CNT_vect){
	++g_seconds;
	RTC_CNT = 0;
	RTC_INTFLAGS = RTC_CMP_bm;
}

ISR(TCB0_INT_vect){
	++g_Accel_Read_Flag;
	++g_LoRa_Check_Flag;
	++g_Motor_Run_Flag;
	++g_BAR_Read_Flag;
	++g_Attitude_Observer_Update_Flag;
	++g_MAG_Read_Flag;
	++g_MRAC_Flag;
	++g_LQR_Flag;
	++g_Print_Flag;
	TCB0_INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB1_INT_vect){
	++g_Attitude_Observer_Predict_Flag;
	TCB1_INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB2_INT_vect){
	++g_Gyro_Read_Flag;
	TCB2_INTFLAGS = TCB_CAPT_bm;
}