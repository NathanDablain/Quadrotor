#include "Ground_Controller.h"

// Keeps track of run time, in seconds
volatile unsigned long g_seconds = 0;

unsigned char Setup(void){
	// Set clock speed, enable interrupts, Initialize ADC, Setup SPI, Setup TWI, Setup Lora, Setup Barometer, Setup SSD
	if (RSTCTRL_RSTFR & RSTCTRL_PORF_bm){Delay(100000);} // Necessary to stabilize IC's on a cold start
	// Sets CPU clock to 24 MHz
	_PROTECTED_WRITE (CLKCTRL_OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc); 
	// Wait for clock to stabilize
	while(!(CLKCTRL_MCLKSTATUS & CLKCTRL_OSCHFS_bm)); 
	Setup_SPI();
	Setup_TWI();
	unsigned char LoRa_setup_status = Setup_LoRa();
	unsigned char BAR_setup_status = Setup_Bar();
	unsigned char SSD_0_setup_status = Setup_SSD(0);
	unsigned char SSD_1_setup_status = Setup_SSD(1);
	Setup_ADC();
	Setup_Timers();
	Setup_Buttons();
	sei();
		
	// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
	//								    LoRa	BAR     SSD1    SSD0
	unsigned char Setup_Bitmask = SSD_0_setup_status | (SSD_1_setup_status<<1) | (BAR_setup_status<<2) | (LoRa_setup_status<<3);

	return Setup_Bitmask;
}

int main(void){
	unsigned char Setup_Bitmask = Setup();
	
	Dial D_Height = {.ID = Height_Dial, .window_left = -170, .window_right = 170};
	Dial D_East = {.ID = East_Dial, .window_left = -170, .window_right = 170};
	Dial D_North = {.ID = North_Dial, .window_left = -170, .window_right = 170};
	Uplink up_link = {Standby, 0.0, 0.0, 0.0, 0.0};
	Downlink down_link = {0, 0};
	Downlink_Reponse_Codes Downlink_Status = No_response;
	unsigned char ID_index = 0;
	
	if (Setup_Bitmask == SETUP_SUCCESS){
		while (1) {
		
			// Read ADCs
			if (g_ADC_Flag >= 2){
				g_ADC_Flag = 0;
				up_link.Desired_altitude = Set_dial_window(&D_Height);
				up_link.Desired_east = Set_dial_window(&D_East);
				up_link.Desired_north = Set_dial_window(&D_North);
			}
			// Set LEDs
		
			// Check if a button has been pressed
			if (g_Button_Read_Flag >= 5) Set_Desired_Status(&up_link, &down_link);
		
			// Read Barometer
			if (g_BAR_Read_Flag >= 3) Read_Bar(&up_link.Pressure_altitude);	
			
			// Write Displays
			if (g_Print_Flag >= 40)	Print_Displays(&D_Height, &D_North, &D_East, &up_link, &down_link, Downlink_Status);
		
			// Check LoRa downlink for flight controller and tracking status
			if (g_LoRa_Check_Flag){
				unsigned char data_available = Check_For_Message();
				if (data_available >= DOWNLINK_SIZE){
					Downlink_Status = Receive_Downlink(&down_link, ID_index, data_available);
				}
			}
	
			// Send LoRa uplink once a second, update ID to check
			if (g_LoRa_Uplink_Flag) ID_index = Send_Uplink(&up_link, &Downlink_Status);
		}
	}
}

ISR(PORTC_PORT_vect){
	if (!(PORTC_IN & (1<<1))) g_Button0_Flag = 1;
	if (!(PORTC_IN & (1<<2))) g_Button1_Flag = 1;
	PORTC_INTFLAGS |= PIN1_bm;
}

ISR(TCB0_INT_vect){
	++g_ADC_Flag;
	++g_Print_Flag;
	++g_Button_Read_Flag;
	++g_BAR_Read_Flag;
	TCB0_INTFLAGS = TCB_CAPT_bm;
}

ISR(RTC_CNT_vect){
	++g_seconds;
	++g_LoRa_Uplink_Flag;
	RTC_CNT = 0;
	RTC_INTFLAGS = RTC_CMP_bm;
}