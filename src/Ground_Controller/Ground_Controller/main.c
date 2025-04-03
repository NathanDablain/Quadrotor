#include "Ground_Controller.h"

int main(void){
	//Setup();
	PORTD_DIR |= (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
	PORTD_OUT |= (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
    while (1) {
    }
}

unsigned char Setup(void){
	// Set clock speed, enable interrupts, Initialize ADC, Setup SPI, Setup TWI, Setup Lora, Setup Barometer, Setup SSD
	if (RSTCTRL_RSTFR & RSTCTRL_PORF_bm){Delay(100000);} // Necessary to stabilize IC's on a cold start
	unsigned char Setup_Bitmask = 0;
	// [7]		[6]		[5]		[4]		[3]		[2]		[1]		[0]
	//										   LoRa	    BAR		SSD
		
	_PROTECTED_WRITE (CLKCTRL_OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc); // Sets CPU clock to 24 MHz
	while(!(CLKCTRL_MCLKSTATUS & CLKCTRL_OSCHFS_bm)); // Wait for clock to stabilize
	Setup_SPI();
	Setup_TWI();
	unsigned char LoRa_setup_status = Setup_LoRa();
	unsigned char BAR_setup_status = Setup_Bar();
	unsigned char SSD_setup_status = Setup_SSD();
	Setup_Bitmask |= SSD_setup_status | (BAR_setup_status<<1) | (LoRa_setup_status<<2);
	//Setup_Timers();
	sei();
		
	return Setup_Bitmask;
}

void Delay(unsigned long long length){
	volatile unsigned long long i = 0;
	while (++i < length);
}