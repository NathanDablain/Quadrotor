#ifndef OLED_H
#define	OLED_H

#include <stdint.h>

typedef struct{
    double dble[6];
} Print_Buffer;

typedef enum{
    OLED_Standby,
    OLED_Fail,
    OLED_Ready,
    OLED_Printing_Page0,
    OLED_Printing_Page1,
    OLED_Printing_Page2,
    OLED_Printing_Page3
} OLED_Machine;

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

#define STANDARD_OLED_DELAY 100

#define OLED_PAGE_LENGTH 128

void Setup_OLED();

void Run_OLED();

uint8_t Write_Display(uint8_t Data_Byte);

uint8_t Write_Display_Double(uint8_t Address_Byte, uint8_t Data_Byte);

uint8_t Clear_Display();

uint8_t Prep_Page(uint8_t page, char *to_print, uint8_t number);

void Send_Pages();

#endif

