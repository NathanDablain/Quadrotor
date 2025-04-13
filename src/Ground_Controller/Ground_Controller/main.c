#include "Ground_Controller.h"

// Tracks when to print
volatile unsigned char g_Print_Flag = 0;
// Tracks when to read the ADC
volatile unsigned char g_ADC_Flag = 0;
// Tracks when button 0 has been pressed
volatile unsigned char g_Button0_Flag = 0;
// Tracks how often to change button status
volatile unsigned char g_Button_Read_Flag = 0;
// Keeps track of run time, in seconds
volatile unsigned char g_seconds = 0;

unsigned char Setup(void){
	// Set clock speed, enable interrupts, Initialize ADC, Setup SPI, Setup TWI, Setup Lora, Setup Barometer, Setup SSD
	if (RSTCTRL_RSTFR & RSTCTRL_PORF_bm){Delay(100000);} // Necessary to stabilize IC's on a cold start
	unsigned char Setup_Bitmask = 0;
	// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
	//								    LoRa	BAR     SSD1    SSD0
	
	_PROTECTED_WRITE (CLKCTRL_OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc); // Sets CPU clock to 24 MHz
	while(!(CLKCTRL_MCLKSTATUS & CLKCTRL_OSCHFS_bm)); // Wait for clock to stabilize
	Setup_SPI();
	Setup_TWI();
	unsigned char LoRa_setup_status = Setup_LoRa();
	unsigned char BAR_setup_status = Setup_Bar();
	unsigned char SSD_0_setup_status = Setup_SSD(0);
	unsigned char SSD_1_setup_status = Setup_SSD(1);
	Setup_Bitmask |= SSD_0_setup_status | (SSD_1_setup_status<<1) | (BAR_setup_status<<2) | (LoRa_setup_status<<3);
	Setup_ADC();
	Setup_Timers();
	Setup_Buttons();
	sei();
	
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
			// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
			//								    LoRa	BAR     SSD1    SSD0
		
			// Read ADCs
			if (g_ADC_Flag >= 2){
				g_ADC_Flag = 0;
				Set_dial_window(&D_Height);
				Set_dial_window(&D_East);
				Set_dial_window(&D_North);
			}
			// Set LEDs
		
			// Check if a button has been pressed
			if (g_Button_Read_Flag >= 5) Set_Status(&up_link);
		
			// Read Barometer
			if (g_BAR_Read_Flag >= 3) Read_Bar(&up_link.Pressure_altitude);	
			
			// Write Displays
			if (g_Print_Flag >= 40)	Print_Displays(&D_Height, &D_North, &D_East, &up_link, Downlink_Status);
		
			// Check LoRa, downlink drone calibration and tracking status
			if (g_LoRa_Check_Flag){
				unsigned char data_available = Check_For_Message();
				if (data_available >= DOWNLINK_SIZE){
					Downlink_Status = Receive_Downlink(&down_link, ID_index, data_available);
				}
			}
			// TODO: Create connection between downlink drone status and uplink drone status
			
			// Send Uplink via LoRa once a second, update ID to check
			if (g_LoRa_Uplink_Flag) ID_index = Send_Uplink(&up_link);
		}
	}
}

void Setup_ADC(){
	// Set VDD as ADC voltage reference
	VREF_ADC0REF |= VREF_REFSEL_VDD_gc;
	// Set GND as negative ADC input
	ADC0_MUXNEG |= ADC_MUXNEG_GND_gc;
	// Set oversampling to 16
	ADC0_CTRLB |= ADC_SAMPNUM_ACC64_gc;
	// Set extended sampling time
	ADC0_SAMPCTRL = 100;
	// Enable ADC
	ADC0_CTRLA |= ADC_ENABLE_bm;
}

void Setup_Buttons(){
	PORTC_PIN1CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
}

void Set_dial_window(Dial *dial){
	// Read ADC
	dial->input_FIR[dial->window_marker] = Read_ADC(dial->ID);
	dial->window_marker++;
	if (dial->window_marker >= DIAL_FIR_SIZE){
		dial->window_marker = 0;
		unsigned char mode_occurances = 0;
		unsigned int mode = 0;
		unsigned char counter;
		unsigned int to_look;
		for (unsigned char i = 0; i < DIAL_FIR_SIZE; i++){
			counter = 0;
			to_look = dial->input_FIR[i];
			for (unsigned char j = 0; j < DIAL_FIR_SIZE; j++){
				if (dial->input_FIR[j] == to_look){
					counter++;
				}
			}
			if (counter >= mode_occurances){
				mode_occurances = counter;
				mode = dial->input_FIR[i];
			}
		}
		dial->reading = mode;
		// if within certain threshold, move value and stop
		if (dial->reading < DIAL_LOWER_BOUND){
			dial->window_right -= 1;
			dial->window_left -= 1;
			dial->output = dial->window_left*DIAL_SENS;
		}
		else if (dial->reading > DIAL_UPPER_BOUND){
			dial->window_right += 1;
			dial->window_left += 1;
			dial->output = dial->window_right*DIAL_SENS;
		}
		else {
			signed long temp = dial->reading-DIAL_LOWER_BOUND;
			temp += dial->window_left;
			dial->output = ((float)temp)*DIAL_SENS;
		}
	}

}

void Setup_Timers(){
	//-Setup Real Time Clock for keeping track of total run time-//
	RTC_CTRLA |= RTC_CORREN_bm | RTC_RTCEN_bm;
	RTC_INTCTRL |= RTC_CMP_bm;
	RTC_CMP = 32768;
	//-------Setup Timer/Counter B0 for output compare---------//
	// Generates an interrupt every 5 ms, is used by:
	//	-> Print statements, variable frequency
	TCB0_CTRLA |= TCB_ENABLE_bm | TCB_CLKSEL_DIV2_gc; // Enables timer, uses main clock with a prescaler of two
	TCB0_INTCTRL |= TCB_CAPT_bm; // Enables interrupt on capture
	TCB0_CCMP = 60000; // Value at which timer generates interrupt and resets
	//--------------------------------------------------------//
}

unsigned int Read_ADC(Dial_ID dial){
	unsigned long ADC_result;
	// Clear MUX 
	ADC0_MUXPOS &= 0x80;
	// Set MUX
	switch (dial){	
		case Height_Dial:
			ADC0_MUXPOS |= ADC_MUXPOS_AIN0_gc;
			break;
		case East_Dial:
			ADC0_MUXPOS |= ADC_MUXPOS_AIN1_gc;
			break;
		case North_Dial:
			ADC0_MUXPOS |= ADC_MUXPOS_AIN2_gc;
			break;
	}
	// Begin conversion
	ADC0_COMMAND |= ADC_STCONV_bm;
	// Wait for conversion to finish
	while (!(ADC0_INTFLAGS & ADC_RESRDY_bm));
	ADC_result = ADC0_RES;
	ADC_result >>= 6;
	
	return ADC_result;
}

void Set_Status(Uplink *up_link){
	g_Button_Read_Flag = 0;
	if (g_Button0_Flag){
		g_Button0_Flag = 0;
		switch (up_link->Drone_status){
			case Standby:
				up_link->Drone_status = Calibrating;
				break;
			case Calibrating:
			// Drone can only move out of calibrating status internally
				break;
			case Ready:
				up_link->Drone_status = Flying;
				break;
			case Flying:
				up_link->Drone_status = Landing;
				break;
			case Landing:
				up_link->Drone_status = Ready;
				break;
		}
	}
}

void Delay(unsigned long long length){
	volatile unsigned long long i = 0;
	while (++i < length);
}

void Print_Displays(Dial *D_h, Dial *D_n, Dial *D_e, Uplink *up_link, Downlink_Reponse_Codes Downlink_Status){
	char *Dl_S_renums[5] = {"NO RESPONSE","INCOMPLETE RESPONSE","BAD ID","BAD CHECKSUM","GOOD RESPONSE"};
	char *Dr_S_renums[5] = {"STANDBY","CALIBRATING","READY","FLYING","LANDING"};
	char buffer[3][20];
	g_Print_Flag = 0;
					
	// Printing on display 0 (right)
	unsigned char length_to_print = snprintf(buffer[0], sizeof(buffer[0]), "HEIGHT: %1.2f", D_h->output);
	Print_Page(0, buffer[0], length_to_print, 0);
	length_to_print = snprintf(buffer[1], sizeof(buffer[1]), "EA5T: %1.2f", D_e->output);
	Print_Page(1, buffer[1], length_to_print, 0);
	length_to_print = snprintf(buffer[2], sizeof(buffer[2]), "NORTH: %1.2f", D_n->output);
	Print_Page(2, buffer[2], length_to_print, 0);
					
	// Printing on display 1 (left)
	Print_Page(0, Dr_S_renums[up_link->Drone_status], strlen(Dr_S_renums[up_link->Drone_status]), 1);
	Print_Page(1, Dl_S_renums[Downlink_Status], strlen(Dl_S_renums[Downlink_Status]), 1);
}

ISR(PORTC_PORT_vect){
	g_Button0_Flag = 1;
	PORTC_INTFLAGS |= PIN1_bm;
}

ISR(TCB0_INT_vect){
	++g_ADC_Flag;
	++g_Print_Flag;
	++g_Button_Read_Flag;
	++g_BAR_Read_Flag;
	++g_LoRa_Check_Flag;
	TCB0_INTFLAGS = TCB_CAPT_bm;
}

ISR(RTC_CNT_vect){
	++g_seconds;
	++g_LoRa_Uplink_Flag;
	RTC_CNT = 0;
	RTC_INTFLAGS = RTC_CMP_bm;
}