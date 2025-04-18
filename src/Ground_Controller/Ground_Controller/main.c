#include "Ground_Controller.h"

// Tracks when to print
volatile unsigned char g_Print_Flag = 0;
// Tracks when to read the ADC
volatile unsigned char g_ADC_Flag = 0;
// Tracks when button 0 has been pressed
volatile unsigned char g_Button0_Flag = 0;
// Tracks how often to change button status
volatile unsigned char g_Button_Read_Flag = 0;

int main(void){
	unsigned char Setup_Bitmask = Setup();
	Dial D_Height;
	D_Height.ID = Height_Dial;
	D_Height.window_left = -170;
	D_Height.window_right = 170;
	Dial D_East;
	D_East.ID = East_Dial; 
	D_East.window_left = -170;
	D_East.window_right = 170;
	Dial D_North;
	D_North.ID = North_Dial;
	D_North.window_left = -170;
	D_North.window_right = 170;
	Button B0 = {Standby, 1, 'C'};
    while (1) {
		// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
		//								    LoRa	BAR     SSD1    SSD0
	
		// Read ADCs
		if (g_ADC_Flag >= 2){
			Set_dial_window(&D_Height);
			Set_dial_window(&D_East);
			Set_dial_window(&D_North);
		}
		// Set LEDs
	
		// Interrupts for buttons
		if (g_Button_Read_Flag >= 5){
			g_Button_Read_Flag = 0;
			if (g_Button0_Flag){
				g_Button0_Flag = 0;
				switch (B0.Drone_status){
					case Standby:
						B0.Drone_status = Calibrating;
						break;
					case Calibrating:
					// Drone can only move out of calibrating status internally
						break;
					case Ready:
						B0.Drone_status = Flying;
						break;
					case Flying:
						B0.Drone_status = Landing;
						break;
					case Landing:
						B0.Drone_status = Ready;
						break;
				}
			}
		}
	
		// Read Barometer
	
		// Write Displays
		if (g_Print_Flag >= 40){
			g_Print_Flag = 0;
			char buffer[4][20];
			
			// Printing on display 0 (right)
			unsigned char length_to_print = snprintf(buffer[0], sizeof(buffer[0]), "HEIGHT: %d, %1.2f", D_Height.reading, D_Height.output);
			Print_Page(0, buffer[0], length_to_print, 0);
			length_to_print = snprintf(buffer[1], sizeof(buffer[1]), "EA5T: %d, %1.2f", D_East.reading, D_East.output);
			Print_Page(1, buffer[1], length_to_print, 0);
			length_to_print = snprintf(buffer[2], sizeof(buffer[2]), "NORTH: %d, %1.2f", D_North.reading, D_North.output);
			Print_Page(2, buffer[2], length_to_print, 0);
			
			// Printing on display 1 (left)
			length_to_print = snprintf(buffer[3], sizeof(buffer[3]), "%d, %d", B0.Drone_status, g_Button0_Flag);
			Print_Page(0, buffer[3], length_to_print, 1);
		}
	
		// Write LoRa
    }
}

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
		case North_Dial:
			ADC0_MUXPOS |= ADC_MUXPOS_AIN1_gc;
			break;
		case East_Dial:
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

void Delay(unsigned long long length){
	volatile unsigned long long i = 0;
	while (++i < length);
}

ISR(PORTC_PORT_vect){
	g_Button0_Flag = 1;
	PORTC_INTFLAGS |= PIN1_bm;
}

ISR(TCB0_INT_vect){
	++g_ADC_Flag;
	++g_Print_Flag;
	++g_Button_Read_Flag;
	TCB0_INTFLAGS = TCB_CAPT_bm;
}