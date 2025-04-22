#include "Peripherals.h"

volatile unsigned char g_ADC_Flag = 0;
volatile unsigned char g_Button0_Flag = 0;
volatile unsigned char g_Button_Read_Flag = 0;

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

float Set_dial_window(Dial *dial){
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

	return dial->output;
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