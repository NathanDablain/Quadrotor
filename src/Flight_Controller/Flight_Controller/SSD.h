#ifndef SSD_H
#define SSD_H

// Macros
// Thanks to https://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html
// 0x80 -> writes single command byte
// 0x00 -> writes stream of command bytes
// 0xC0 -> writes single data byte
// 0x40 -> writes stream of data bytes
// Display is 128 pixels wide, four pages tall, with the second four pages nestled between the first four
// I2C address of device
#define SSD_ADR 0x3C
// Important registers for initialization, in order they should be called
#define SSD_DISPLAY_OFF 0xAE
#define SSD_MULTIPLEX_RATIO 0xA8
#define SSD_DISPLAY_OFFSET 0xD3
#define SSD_DISPLAY_START_LINE 0x40
#define SSD_SEGMENT_REMAP 0xA0
#define SSD_COM_OUTPUT_SCAN_DIRECTION 0xC0
#define SSD_COM_PINS_CONFIGURATION 0xDA
#define SSD_CONTRAST_CONTROL 0x81
#define SSD_ENTIRE_DISPLAY_RAM 0xA4
#define SSD_NORMAL_DISPLAY 0xA6
#define SSD_OSC_FREQUENCY 0xD5
#define SSD_CHARGE_PUMP 0x8D
#define SSD_DISPLAY_ON 0xAF
#define SSD_ENTIRE_DISPLAY_ON 0xA5

#define SSD_PAGE0 0xB0
#define SSD_PAGE1 0xB1
#define SSD_PAGE2 0xB2
#define SSD_PAGE3 0xB3
#define SSD_PAGE4 0xB4
#define SSD_PAGE5 0xB5
#define SSD_PAGE6 0xB6
#define SSD_PAGE7 0xB7

// Functions
unsigned char
	Setup_SSD(),
	Write_Display(unsigned char Data_Byte),
	Write_Display_Double(unsigned char Address_Byte, unsigned char Data_Byte),
	Write_Character(char Character_to_write),
	Clear_Display(),
	Print_Page(unsigned char page, char *to_print, unsigned char number);

void Print_Output(float *Desired_Thrust, float Desired_Moments[3], unsigned int Motor_Throttles[4]);

#endif