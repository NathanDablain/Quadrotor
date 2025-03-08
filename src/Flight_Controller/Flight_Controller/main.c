#include "main.h"

// Global variables
static volatile unsigned char g_Motor_Run_Flag = 0;
static volatile unsigned char g_MAG_Read_Flag = 0;
static volatile unsigned char g_BAR_Read_Flag = 0;
static volatile unsigned char g_Attitude_Observer_Run_Flag = 0;
static volatile unsigned char g_Print_Flag = 0;
static volatile unsigned char g_IMU_Read_Flag = 0;
static volatile unsigned char g_LoRa_Flag = 0;
volatile unsigned long g_seconds = 0;

int main(){
	unsigned char Setup_Bitmask = Setup();
	if ((Setup_Bitmask & NAV_SENSORS_bm) == NAV_SENSORS_bm){
		while(1){
			Run(Setup_Bitmask);
		}
	}
	
	return 0;
}

// Function Definitions

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

void Run(unsigned char Setup_Bitmask){
	static States Drone = {0};
	static States Reference = {0};
	static unsigned int motor_throttles[4] = {0};
	
	// NAVIGATION //
	static unsigned char Navigation_Bitmask = 0;
	// This section handles all sensor timing and reading, will populate a bit mask of the sensor statuses for guidance and control functions
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
	
	if (g_LoRa_Flag){
		g_LoRa_Flag = 0;
		motor_throttles[0] = Read_LoRa(&Reference);
		char buffer[5] = {0};
		unsigned char length_to_print = snprintf(buffer, sizeof(buffer), "%d", motor_throttles[0]);
		Print_Page(3, buffer, length_to_print);
		//Navigation_Bitmask = SET_BIT(Navigation_Bitmask, NAV_LORA_bp, LoRa_status);
	}
	
	if ((g_Print_Flag >= 50)&&(Setup_Bitmask & (1<<SU_SSD_bp))){
		g_Print_Flag = 0;
		char buffer0[10] = {0};
		char buffer1[10] = {0};
		char buffer2[15] = {0};
		unsigned char length_to_print = snprintf(buffer0, sizeof(buffer0), "%3.2f", Drone.Euler[0]);
		Print_Page(0, buffer0, length_to_print);
		length_to_print = snprintf(buffer1, sizeof(buffer1), "%3.2f", Drone.Euler[1]);
		Print_Page(1, buffer1, length_to_print);
		length_to_print = snprintf(buffer2, sizeof(buffer2), "%3.2f", Drone.Euler[2]);
		Print_Page(2, buffer2, length_to_print);
	}
	
	// GUIDANCE //
	
	if (Navigation_Bitmask & NAV_SENSORS_bm){
		
	}
	
	// CONTROL //
	//if (Navigation_Bitmask & NAV_SENSORS_bm){
		if (g_Motor_Run_Flag >= 2){
			g_Motor_Run_Flag = 0;
			Run_Motors(motor_throttles);
		}
	//}
}

void Delay(unsigned long long length){
	volatile unsigned long long i = 0;
	while (++i<length);
}

ISR(RTC_CNT_vect){
	++g_seconds;
	++g_LoRa_Flag;
	RTC_INTFLAGS = RTC_CMP_bm;
}

ISR(TCB0_INT_vect){
	++g_Motor_Run_Flag;
	++g_Print_Flag;
	++g_BAR_Read_Flag;
	++g_Attitude_Observer_Run_Flag;
	++g_MAG_Read_Flag;
	TCB0_INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB1_INT_vect){
	++g_IMU_Read_Flag;
	TCB1_INTFLAGS = TCB_CAPT_bm;
}