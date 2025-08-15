#include "Flight_Controller.h"

// Initialize all global variables here //
volatile unsigned char g_positive_coms_watchdog = 0;
volatile unsigned long g_seconds = 0;
volatile unsigned char g_print_flag = 0;
volatile unsigned char g_LoRa_Check_Flag = 0;
volatile unsigned char g_Motor_Power_Flag = 0;
volatile unsigned char g_Guidance_Flag = 0;
volatile unsigned char g_Altitude_Control_Flag = 0;
volatile unsigned char g_Motor_Run_Flag = 0;
volatile unsigned char g_Motor_Cal_Flag = 0;
volatile unsigned int g_Motor_Throttles[4] = {0};
volatile unsigned char g_BAR_Read_Flag = 0;
volatile unsigned char g_Accel_Read_Flag = 0;
volatile unsigned char g_Gyro_Read_Flag = 0;
volatile unsigned char g_MAG_Read_Flag = 0;
volatile unsigned char g_Attitude_Observer_Update_Flag = 0;
volatile unsigned char g_Attitude_Observer_Predict_Flag = 0;


unsigned char Setup(){
	Setup_Pins();
	if (RSTCTRL_RSTFR & RSTCTRL_PORF_bm){Delay(100000);} // Necessary to stabilize IC's on a cold start
	unsigned char Setup_Bitmask = 0;
	// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
	//					SSD	   LoRa	    MAG		IMU		BAR		GPS
	
	_PROTECTED_WRITE (CLKCTRL_OSCHFCTRLA, (CLKCTRL_FRQSEL_24M_gc|CLKCTRL_AUTOTUNE_bm)); // Sets CPU clock to 24 MHz
	while(!(CLKCTRL_MCLKSTATUS & CLKCTRL_OSCHFS_bm)); // Wait for clock to stabilize
	unsigned char GPS_setup_status = Setup_GPS();
	Setup_SPI();
	Setup_TWI();
	Setup_ADC();
	unsigned char LoRa_setup_status = Setup_LoRa();
	unsigned char MAG_setup_status = Setup_Mag();
	unsigned char IMU_setup_status = Setup_IMU();
	unsigned char BAR_setup_status = Setup_Bar();
	unsigned char SSD_setup_status = Setup_SSD();
	Setup_Bitmask |= (GPS_setup_status<<NAV_GPS_bp) | (BAR_setup_status<<NAV_BAR_bp) | (IMU_setup_status<<NAV_IMU_bp) | (MAG_setup_status<<NAV_MAG_bp)
	| (LoRa_setup_status<<NAV_LORA_bp) | (SSD_setup_status<<SU_SSD_bp);
	Run_Motors(1);
	Setup_Timers();
	sei();
	return Setup_Bitmask;
}

int main(){
	unsigned char Setup_Bitmask = Setup();
	// If the sensors we need for navigation initialized successfully, enter main loop
	if ((Setup_Bitmask & NAV_SENSORS_bm) == NAV_SENSORS_bm){
		// Initialize data structures
		const Drone_Constants Constants = Initialize_Drone_Constants();
		// Drone-> tracks the current drone states
		States Drone = {0};
		// Desired-> tracks the desired drone states issued by the ground controller
		Reference Desired_States = {0};
		// Commanded-> tracks the states the autopilot is tracking to after the desired states are fed through the guidance functions
		Reference Commanded_States = {0};
		// up_link-> contains the last information sent to the drone via LoRa uplink
		Uplink up_link = {0};
		// Flight_Controller_Status-> Controls the mode of operation the drone is in, changed by uplinks from the ground controller
		FC_Status Flight_Controller_Status = Standby;
		// Holds sensor calibration data 
		Calibration_Data cal_data = {0};
		// Desired_Thrust-> Controlled by state feedback, in units of N
		float Desired_Thrust = 0.0;
		// Desired_Moments-> Controlled by PID, moment order is: body x, body y, body z, in units of N-m
		float Desired_Moments[3] = {0};
		unsigned char reset = 0;
		while(1){
			//--------------Common code--------------//
			// LoRa
			if (g_LoRa_Check_Flag>=2) Run_LORA(&up_link, &Flight_Controller_Status);

			// Printing
			if (g_print_flag &&(Setup_Bitmask & (1<<SU_SSD_bp))){
				g_print_flag = 0;
				char buffer[4][20] = {0};
				unsigned char length_to_print = snprintf(buffer[0], sizeof(buffer[0]), "%4.2f, %4.2f, %4.2f", Drone.Euler[0], Drone.Euler[1], Drone.Euler[2]);
				Print_Page(0, buffer[0], length_to_print);
				length_to_print = snprintf(buffer[1], sizeof(buffer[1]), "%4.2f , %4.2f",-Drone.Position_NED[2],Desired_Thrust);
				Print_Page(1, buffer[1], length_to_print);
				ATOMIC_BLOCK(ATOMIC_FORCEON){
					length_to_print = snprintf(buffer[2], sizeof(buffer[2]), "B %d L %d", g_Motor_Throttles[0],g_Motor_Throttles[1]);
					Print_Page(2, buffer[2], length_to_print);
					length_to_print = snprintf(buffer[3], sizeof(buffer[3]), "R %d F %d",g_Motor_Throttles[2],g_Motor_Throttles[3]);
					Print_Page(3, buffer[3], length_to_print);
				}
			}
			
			// GPS -> Check when full message is received
			if (g_GPS_Read_Flag && (Setup_Bitmask & (1<<NAV_GPS_bp))) Read_GPS(&Drone, &cal_data);

			// Barometer -> check 100Hz, samples at 75Hz
			if (g_BAR_Read_Flag >= 2) Read_Bar(&Drone, &cal_data, up_link.Base_altitude);

			// Magnetometer -> check at 200 Hz, samples at 50Hz
			if (g_MAG_Read_Flag) Read_Mag(&Drone, &cal_data);
			
			// Accelerometer -> check at 200 Hz, samples at 52Hz
			if (g_Accel_Read_Flag) Read_Accel(&Drone);

			// Gyro -> Check at 1666Hz, samples at 416Hz
			if (g_Gyro_Read_Flag) Read_Gyro(&Drone, &cal_data); 

			// Attitude Observer update -> 50Hz
			if (g_Attitude_Observer_Update_Flag >= 4) Attitude_Observer_Update(&Drone);
				
			// Attitude Observer predict -> 400Hz
			if (g_Attitude_Observer_Predict_Flag >= 4) Attitude_Observer_Predict(&Drone);

			//--------------Status code-------------//
			if (Flight_Controller_Status == Standby){
				// Initialize or reset calibration data
				if (reset == 0){
					memset(&cal_data, 0, sizeof(cal_data));
					Drone.Latitude = -1;
					Drone.Longitude = -1;
					ATOMIC_BLOCK(ATOMIC_FORCEON){
						memset((unsigned int*)g_Motor_Throttles, 0, sizeof(g_Motor_Throttles));
					}
					reset++;
				}
			}
			else if (Flight_Controller_Status == Calibrating){
				reset = 0;
				
				if (cal_data.bar_cal_status == 0) Calibrate_Bar(&Drone, &cal_data, up_link.Base_altitude);
				
				if (cal_data.mag_cal_status == 0) Calibrate_Mag(&Drone, &cal_data);
				
				if (cal_data.imu_cal_status == 0) Calibrate_IMU(&Drone, &cal_data);
				
				if (Sample_ADC() > MOTOR_VOLTAGE_THRESHOLD){
					cal_data.motor_cal_status = 1;
					g_Motor_Power_Flag = 1;
				}
				//if ((cal_data.motor_cal_status == 0) && (Sample_ADC() > MOTOR_VOLTAGE_THRESHOLD)) Calibrate_Motors(&cal_data);

				if (cal_data.bar_cal_status && cal_data.mag_cal_status && cal_data.imu_cal_status && cal_data.motor_cal_status) Flight_Controller_Status = Ready;
	
			}
			else if (Flight_Controller_Status == Ready){

			}
			else {
			//------------Guidance and Control functions-------------//
				if (Flight_Controller_Status == Flying){
					if (up_link.Desired_altitude < 5.0){
						Desired_States.Position_NED[2] = -up_link.Desired_altitude;
					}
				}
				else if (Flight_Controller_Status == Landing){
					Desired_States.Position_NED[2] = 1.0;
					if (Drone.Position_NED[2] >= 0){
						Flight_Controller_Status = Standby;
						Desired_Thrust = 0.0;
						memset(Desired_Moments, 0, sizeof(Desired_Moments));
					}
				}
				
				if (g_Guidance_Flag) Run_Guidance(&Desired_States, &Commanded_States);
				
				// Altitude Controller Runs at 100 Hz, updates desired thrust
				if (g_Altitude_Control_Flag >= 2) Desired_Thrust = Altitude_Control(-Drone.Position_NED[2], -Commanded_States.Position_NED[2], &Constants);

				// Euler angle controller and throttle updates run at 200 Hz, PWM sent to ESC at 400 Hz in TCD interrupt
				if (g_Motor_Run_Flag){
					g_Motor_Run_Flag = 0;
					Euler_Control(Drone.Euler, Commanded_States.Euler, Desired_Moments, Desired_Thrust, &Constants);
					Set_throttles(Desired_Thrust, Desired_Moments, &Constants);
					Safety_Check(&Drone, &Flight_Controller_Status);
				}
			}
		}

	}
	
	return 0;
}

void Setup_ADC(){
	// Set VDD as ADC voltage reference
	VREF_ADC0REF |= VREF_REFSEL_VDD_gc;
	// Set GND as negative ADC input
	ADC0_MUXNEG |= ADC_MUXNEG_GND_gc;
	// Set oversampling to 64 -> becomes 10 bit ADC
	ADC0_CTRLB |= ADC_SAMPNUM_ACC64_gc;
	// Set extended sampling time
	ADC0_SAMPCTRL = 100;
	// Set MUX position
	ADC0_MUXPOS |= ADC_MUX_ESC;
	// Enable ADC
	ADC0_CTRLA |= ADC_ENABLE_bm;
}

unsigned int Sample_ADC(){
	// Returns the voltage read in milli volts
	// Begin conversion
	ADC0_COMMAND |= ADC_STCONV_bm;
	// Wait for conversion to finish
	while (!(ADC0_INTFLAGS & ADC_RESRDY_bm));
	unsigned int ADC_result = ADC0_RES;
	ADC_result >>= 6;
	unsigned int voltage_mv = (ADC_result*15) + ADC_result;
	voltage_mv = voltage_mv + (((voltage_mv*7)/10)/5);
	return voltage_mv;
}

void Setup_Timers(){
	//-Setup Real Time Clock for keeping track of total run time-//
	RTC_CTRLA |= RTC_CORREN_bm | RTC_RTCEN_bm;
	RTC_INTCTRL |= RTC_CMP_bm;
	RTC_CMP = 32768;
	//----------------------------------------------------------//
	//--------Setup Timer/Counter A0 for output compare---------//
	// Is triggered every 2.5 ms, is used by:
	//  -> Motors
	TCA0_SINGLE_CTRLA |= TCA_SINGLE_CLKSEL_DIV2_gc;
	TCA0_SINGLE_INTCTRL |= TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm;
	//---------------------------------------------------------//
	//-------Setup Timer/Counter B0 for output compare---------//
	// Generates an interrupt every 5 ms, is used by:
	//	-> Magnetometer running at 100 Hz
	//  -> Barometer running at 75 Hz
	//	-> Attitude observer running at 25 Hz
	//	-> Print statements, variable frequency
	//  -> Accelerometer
	TCB0_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV2_gc; // Enables timer, uses main clock with a prescaler of two
	TCB0_INTCTRL |= TCB_CAPT_bm; // Enables interrupt on capture
	TCB0_CCMP = 60000; // Value at which timer generates interrupt and resets
	//-------------------------------------------------------//
	//-------Setup Timer/Counter B1 for output compare--------//
	// Generates an interrupt every 0.6002 ms, is used by:
	//	-> Gyro running at 1666 Hz
	//  -> Observer running at 416 Hz
	TCB1_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV1_gc;
	TCB1_INTCTRL |= TCB_CAPT_bm;
	TCB1_CCMP = 14405;
#if defined(AVR128DB48)
	//-------------------------------------------------------//
	//-------Setup Timer/Counter B3 for output compare-------//
	// Generates an interrupt every 286 us (3500 Hz), is used by:
	//  -> Oneshot protocol setting motor speed
	TCB2_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV1_gc;
	TCB2_INTCTRL |= TCB_CAPT_bm;
	TCB2_CCMP = 5900;
	//----------------------------------------------------------//
	//--------Setup Timer/Counter A1 for output compare---------//
	// Is triggered every 2.5 ms, is used by:
	//  -> Motors
	TCA1_SINGLE_CTRLA |= TCA_SINGLE_CLKSEL_DIV2_gc;
	TCA1_SINGLE_INTCTRL |= TCA_SINGLE_CMP0_bm;
#elif defined(AVR64DA28)
	//-------------------------------------------------------//
	//-------Setup Timer/Counter B2 for output compare-------//
	// Is triggered every 2.5 ms, is used by:
	//  -> Motors
	TCB2_CTRLA |= TCB_CLKSEL_DIV2_gc;
	TCB2_INTCTRL |= TCB_CAPT_bm;
	//----------------------------------------------------------//
	//--------Setup Timer/Counter D for output compare---------//
	// Generates an interrupt every 2.5 ms (400 Hz), is used by:
	//  -> Oneshot protocol setting motor speed
	// In one ramp mode goes CMPASET->CMPACLR->CMPBSET->COMPBCLR
	TCD0_CTRLA |= TCD_CNTPRES_DIV32_gc;
	TCD0_CMPBCLR = 1875;
	TCD0_INTCTRL |= TCD_OVF_bm;
	while(!(TCD0_STATUS & TCD_ENRDY_bm));
	TCD0_CTRLA |= TCD_ENABLE_bm;
#endif
}

ISR(RTC_CNT_vect){
	++g_seconds;
	++g_print_flag;
	++g_positive_coms_watchdog;
	RTC_CNT = 0;
	RTC_INTFLAGS = RTC_CMP_bm;
}

ISR(TCB0_INT_vect){
	++g_Motor_Run_Flag;
	++g_Accel_Read_Flag;
	++g_LoRa_Check_Flag;
	++g_BAR_Read_Flag;
	++g_Attitude_Observer_Update_Flag;
	++g_MAG_Read_Flag;
	++g_Altitude_Control_Flag;
	++g_Guidance_Flag;
	TCB0_INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB1_INT_vect){
	++g_Attitude_Observer_Predict_Flag;
	++g_Gyro_Read_Flag;
	TCB1_INTFLAGS = TCB_CAPT_bm;
}

#if defined(AVR128DB48)
ISR(TCB2_INT_vect){
	if (g_Motor_Power_Flag) Run_Motors(0);
	TCB2_INTFLAGS = TCB_CAPT_bm;
}
#elif defined(AVR64DA28)
ISR(TCD0_OVF_vect){
	if (g_Motor_Power_Flag) Run_Motors(0);
	g_Motor_Cal_Flag = 1;
	TCD0_INTFLAGS = TCD_OVF_bm;
}
#endif