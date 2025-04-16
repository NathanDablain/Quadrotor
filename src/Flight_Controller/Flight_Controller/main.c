#include "Flight_Controller.h"

static volatile unsigned char 
	g_Motor_Run_Flag,
	g_MAG_Read_Flag,
	g_BAR_Read_Flag, 
	g_Attitude_Observer_Run_Flag, 
	g_IMU_Read_Flag,
	g_MRAC_Flag,
	g_LQR_Flag;
volatile unsigned long g_seconds = 0;
	
unsigned char Setup(){
	if (RSTCTRL_RSTFR & RSTCTRL_PORF_bm){Delay(100000);} // Necessary to stabilize IC's on a cold start
	unsigned char Setup_Bitmask = 0;
	// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
	//					SSD	   LoRa	    MAG		IMU		BAR		GPS
	
	_PROTECTED_WRITE (CLKCTRL_OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc); // Sets CPU clock to 24 MHz
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
	// Generates an interrupt every 4.807 ms, is used by:
	//	-> IMU running at 208 Hz
	TCB1_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV2_gc;
	TCB1_INTCTRL |= TCB_CAPT_bm;
	TCB1_CCMP = 57693;
	//-------------------------------------------------------//
}

int main(){
	unsigned char Setup_Bitmask = Setup();
	// If the sensors we need for navigation initialized sucessfully, enter main loop
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
		// Motor_Throttles-> Values from 0-1000 with 1000 being max throttle, motor order is: back, left, right, front
		unsigned int Motor_Throttles[4];
		// Desired_Thrust-> Controlled by a Model Reference Adaptive Controller (MRAC), in units of N
		float Desired_Thrust;
		// Desired_Moments-> Controlled by 3 PIDs, moment order is: body x, body y, body z, in units of N-m
		float Desired_Moments[3];
		// Bits track when a sensor has finished calibrating
		unsigned char Calibration_Bitmask = 0;
		unsigned char Calibration_Loop_Exit_Flag = 0;
		// Calibration loop
		while(1){
			if (g_LoRa_Check_Flag){
				unsigned char data_available = Check_For_Message();
				if (data_available >= UPLINK_SIZE){
					Receive_Uplink(&up_link, &down_link, &Flight_Controller_Status, data_available);
					if (Calibration_Loop_Exit_Flag) break;
				}
			}
			if (Flight_Controller_Status == Calibrating){
				//if (g_BAR_Read_Flag >= 3)

				if (!(Calibration_Bitmask & ~CALIBRATION_COMPLETE)){
					// Respond to ground controller on next uplink that calibration is complete and the drone is ready to fly
					Flight_Controller_Status = Ready;
					down_link.Calibration_Status = 1;
					Calibration_Loop_Exit_Flag = 1;
				} 
			}
		}
		// Ready, Flying, and Landing loop
		unsigned char Navigation_Bitmask = 0;
		while(1){
			// NAVIGATION //
			// This section handles all sensor timing and reading, will populate a bit mask of the sensor statuses to indicate when calibration is complete
			// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
			//				           LoRa	    MAG		IMU		BAR		GPS

			if (g_GPS_Read_Flag){
				g_GPS_Read_Flag = 0;
				unsigned char GPS_status = Read_GPS(&Drone);
				Navigation_Bitmask = SET_BIT(Navigation_Bitmask, NAV_GPS_bp, GPS_status);
			}

			if (g_BAR_Read_Flag >= 3){
				g_BAR_Read_Flag = 0;
				unsigned char BAR_status = Read_Bar(&Drone);
				Navigation_Bitmask = SET_BIT(Navigation_Bitmask, NAV_BAR_bp, BAR_status);
			}

			if (g_MAG_Read_Flag >= 2){
				g_MAG_Read_Flag = 0;
				unsigned char MAG_status = Read_Mag(&Drone);
				Navigation_Bitmask = SET_BIT(Navigation_Bitmask, NAV_MAG_bp, MAG_status);
			}

			if (g_IMU_Read_Flag){
				g_IMU_Read_Flag = 0;
				unsigned char IMU_status = Read_IMU(&Drone);
				Navigation_Bitmask = SET_BIT(Navigation_Bitmask, NAV_IMU_bp, IMU_status);
			}

			if (g_Attitude_Observer_Run_Flag >= 8){
				g_Attitude_Observer_Run_Flag = 0;
				Observer(&Drone);
			}

			if (g_LoRa_Check_Flag){
				unsigned char data_available = Check_For_Message();
				if (data_available >= UPLINK_SIZE){
					//Uplink up_link;
					//unsigned char uplink_status = Receive_Uplink(&up_link);
				}
			}

			if (g_Print_Flag&&(Setup_Bitmask & (1<<SU_SSD_bp))) Print_Output(&Desired_Thrust, Desired_Moments, Motor_Throttles);

			if (Navigation_Bitmask & NAV_SENSORS_bm){
				// GUIDANCE //
				if (g_MRAC_Flag >= 50){
					g_MRAC_Flag = 0;
					Desired_Thrust = 4.32; //Height_MRAC(&Drone, -Reference.Position_NED[2]);
				}
				if (g_LQR_Flag >= 8){
					g_LQR_Flag = 0;
					Attitude_LQR(&Drone, &Reference);
				}
				// CONTROL //
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
	++g_Print_Flag;
	//++g_LoRa_Flag;
	RTC_CNT = 0;
	RTC_INTFLAGS = RTC_CMP_bm;
}

ISR(TCB0_INT_vect){
	++g_LoRa_Check_Flag;
	++g_Motor_Run_Flag;
	//++g_Print_Flag;
	++g_BAR_Read_Flag;
	++g_Attitude_Observer_Run_Flag;
	++g_MAG_Read_Flag;
	++g_MRAC_Flag;
	++g_LQR_Flag;
	TCB0_INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB1_INT_vect){
	++g_IMU_Read_Flag;
	TCB1_INTFLAGS = TCB_CAPT_bm;
}