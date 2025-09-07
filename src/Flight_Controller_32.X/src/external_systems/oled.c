#include <xc.h>
#include <stdint.h>
#include "global_variables.h"
#include "i2c.h"
#include "oled.h"
#include "pins.h"

uint8_t Setup_OLED(){
	uint8_t Setup_status = 1;
	// From data sheet - order of software tasks to initialize display
	//Setup_status &= Write_Display(SSD_DISPLAY_OFF);
	// 1. Set MUX ratio -> 0xA8, 0x3F
	Setup_status &= Write_Display_Double(SSD_MULTIPLEX_RATIO, 0x3F);
	// 2. Set Display Offset -> 0xD3, 0x00
	Setup_status &= Write_Display_Double(SSD_DISPLAY_OFFSET, 0x00);
	// 3. Set Display Start Line -> 0x40
	Setup_status &= Write_Display(SSD_DISPLAY_START_LINE);
	// 4. Set Segment re-map -> 0xA0
	Setup_status &= Write_Display(SSD_SEGMENT_REMAP);
	// 5. Set COM Output Scan Direction -> 0xC0
	Setup_status &= Write_Display(SSD_COM_OUTPUT_SCAN_DIRECTION);
	// 6. Set COM Pins hardware configuration -> 0xDA, 02
	Setup_status &= Write_Display_Double(SSD_COM_PINS_CONFIGURATION, 0x02);
	// 7. Set Contrast Control -> 0x81, 0x7F
	Setup_status &= Write_Display_Double(SSD_CONTRAST_CONTROL, 0x7F);
	// 8. Disable Entire Display On -> 0xA4
	Setup_status &= Write_Display(SSD_ENTIRE_DISPLAY_RAM);
	// 9. Set Normal Display -> 0xA6
	Setup_status &= Write_Display(SSD_NORMAL_DISPLAY);
	// 10. Set Oscillator Frequency -> 0xD5, 0x80
	Setup_status &= Write_Display_Double(SSD_OSC_FREQUENCY, 0x80);
	// 11. Enable charge pump regulator -> 0x8D, 0x14
	Setup_status &= Write_Display_Double(SSD_CHARGE_PUMP, 0x14);
	// 12. Display On -> 0xAF
	Setup_status &= Write_Display(SSD_DISPLAY_ON);

	Setup_status &= Clear_Display();
	
	return Setup_status;
}

inline uint8_t Write_Display(uint8_t Data_Byte){
	uint8_t TWI_status = Write_I2C(SSD_ADR, 0x80, &Data_Byte, 1);
	
	return (TWI_status == 4) ? 1 : 0;
}

inline uint8_t Write_Display_Double(uint8_t Address_Byte, uint8_t Data_Byte){
	uint8_t input_data[2] = {Address_Byte, Data_Byte};
	uint8_t TWI_status = Write_I2C(SSD_ADR, 0x00, input_data, 2);
	
	return (TWI_status == 4) ? 1 : 0;
}

uint8_t Write_Character(char Character_to_write){
	// Always leave bottom bit blank, bits order from bottom (7) to top (0)
	const uint8_t SSD_space[3] = {0x00, 0x00, 0x00};
	const uint8_t SSD_dot[3] = {0x00, 0b01000000, 0x00};
	const uint8_t SSD_dash[3] = {0b00001000, 0b00001000, 0b00001000};
	const uint8_t SSD_comma[3] = {0b00100000, 0b01100000, 0b00000000};
	const uint8_t SSD_asterisk[3] = {0b00000010, 0b00000111, 0b00000010};
	const uint8_t SSD_dollar[5] = {0b01001111, 0b01001001, 0b011111111, 0b01001001, 0b01111001};
	const uint8_t SSD_colon[2] = {0b00100100, 0b00100100};
	const uint8_t SSD_0[4] = {0b00111110, 0b01000001, 0b01000001, 0b00111110};
	const uint8_t SSD_1[3] = {0b01000010, 0b01111111, 0b01000000};
	const uint8_t SSD_2[4] = {0b01111001, 0b01001001, 0b01001001, 0b01001111};
	const uint8_t SSD_3[5] = {0b01000001, 0b01001001, 0b01001001, 0b01010101, 0b00110110};
	const uint8_t SSD_4[5] = {0b00010000, 0b00011000, 0b00010100, 0b00010010, 0b01111111};
	const uint8_t SSD_5[4] = {0b01001111, 0b01001001, 0b01001001, 0b01111001};
	const uint8_t SSD_6[5] = {0b00011100, 0b00101010, 0b01001001, 0b00101001, 0b00010000};
	const uint8_t SSD_7[6] = {0b01000001, 0b00100001, 0b00010001, 0b00001001, 0b00000101, 0b00000011};
	const uint8_t SSD_8[5] = {0b00010100, 0b00101010, 0b01001001, 0b00101010, 0b00010100};
	const uint8_t SSD_9[5] = {0b01000110, 0b00101010, 0b00011001, 0b00001010, 0b00000100};
	const uint8_t SSD_A[5] = {0b01111100, 0b00001010, 0b00001001, 0b00001010, 0b01111100};
	const uint8_t SSD_B[5] = {0b01111111, 0b01001001, 0b01001001, 0b01011101, 0b00100010};
	const uint8_t SSD_C[4] = {0b00011100, 0b00100010, 0b01000001, 0b00100010};
	const uint8_t SSD_D[4] = {0b01111111, 0b01000001, 0b00100010, 0b00011100};
	const uint8_t SSD_E[4] = {0b01111111, 0b01001001, 0b01001001, 0b01001001};
	const uint8_t SSD_F[4] = {0b01111111, 0b00010001, 0b00010001, 0b00000001};
	const uint8_t SSD_G[5] = {0b00011100, 0b00100010, 0b01010001, 0b01010001, 0b01110010};
	// SSD_H refers to the header file
	const uint8_t SSD_h[5] = {0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111};
	const uint8_t SSD_I[5] = {0b01000001, 0b01000001, 0b01111111, 0b01000001, 0b01000001};
	const uint8_t SSD_J[5] = {0b00100001, 0b01000001, 0b00100001, 0b00011111, 0b00000001};
	const uint8_t SSD_K[5] = {0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001};
	const uint8_t SSD_L[4] = {0b01111111, 0b01000000, 0b01000000, 0b01000000};
	const uint8_t SSD_M[5] = {0b01111111, 0b00000010, 0b00000100, 0b00000010, 0b01111111};
	const uint8_t SSD_N[7] = {0b01111111, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01111111};
	const uint8_t SSD_O[4] = {0b00111110, 0b01000001, 0b01000001, 0b00111110};
	const uint8_t SSD_P[4] = {0b01111111, 0b00001001, 0b00001001, 0b00000110};
	const uint8_t SSD_Q[6] = {0b00011100, 0b00100010, 0b01000001, 0b01000101, 0b00100010, 0b00011101};
	const uint8_t SSD_R[5] = {0b01111111, 0b00001001, 0b00011001, 0b00100110, 0b01000000};
	// S copies from 5
	const uint8_t SSD_T[5] = {0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001};
	const uint8_t SSD_U[5] = {0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111};
	const uint8_t SSD_V[11] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010, 0b00000001};
	//const uint8_t SSD_W[];
	//const uint8_t SSD_X[];
	const uint8_t SSD_Y[5] = {0b00000001, 0b00000010, 0b01111100, 0b00000010, 0b00000001};
	//const uint8_t SSD_Z[];
	
	const uint8_t *output;
	uint8_t output_size;
	switch (Character_to_write){
		case '0':
		output = SSD_0;
		output_size = sizeof(SSD_0);
		break;
		case '1':
		output = SSD_1;
		output_size = sizeof(SSD_1);
		break;
		case '2':
		output = SSD_2;
		output_size = sizeof(SSD_2);
		break;
		case '3':
		output = SSD_3;
		output_size = sizeof(SSD_3);
		break;
		case '4':
		output = SSD_4;
		output_size = sizeof(SSD_4);
		break;
		case '5':
		output = SSD_5;
		output_size = sizeof(SSD_5);
		break;
		case '6':
		output = SSD_6;
		output_size = sizeof(SSD_6);
		break;
		case '7':
		output = SSD_7;
		output_size = sizeof(SSD_7);
		break;
		case '8':
		output = SSD_8;
		output_size = sizeof(SSD_8);
		break;
		case '9':
		output = SSD_9;
		output_size = sizeof(SSD_9);
		break;
		case 'A':
		output = SSD_A;
		output_size = sizeof(SSD_A);
		break;
		case 'B':
		output = SSD_B;
		output_size = sizeof(SSD_B);
		break;
		case 'C':
		output = SSD_C;
		output_size = sizeof(SSD_C);
		break;
		case 'D':
		output = SSD_D;
		output_size = sizeof(SSD_D);
		break;
		case 'E':
		output = SSD_E;
		output_size = sizeof(SSD_E);
		break;
		case 'F':
		output = SSD_F;
		output_size = sizeof(SSD_F);
		break;
		case 'G':
		output = SSD_G;
		output_size = sizeof(SSD_G);
		break;
		case 'H':
		output = SSD_h;
		output_size = sizeof(SSD_h);
		break;
		case 'I':
		output = SSD_I;
		output_size = sizeof(SSD_I);
		break;
		case 'J':
		output = SSD_J;
		output_size = sizeof(SSD_J);
		break;
		case 'K':
		output = SSD_K;
		output_size = sizeof(SSD_K);
		break;
		case 'L':
		output = SSD_L;
		output_size = sizeof(SSD_L);
		break;
		case 'M':
		output = SSD_M;
		output_size = sizeof(SSD_M);
		break;
		case 'N':
		output = SSD_N;
		output_size = sizeof(SSD_N);
		break;
		case 'O':
		output = SSD_O;
		output_size = sizeof(SSD_O);
		break;
		case 'P':
		output = SSD_P;
		output_size = sizeof(SSD_P);
		break;
		case 'Q':
		output = SSD_Q;
		output_size = sizeof(SSD_Q);
		break;
		case 'R':
		output = SSD_R;
		output_size = sizeof(SSD_R);
		break;
		case 'S':
		output = SSD_5;
		output_size = sizeof(SSD_5);
		break;
		case 'T':
		output = SSD_T;
		output_size = sizeof(SSD_T);
		break;
		case 'U':
		output = SSD_U;
		output_size = sizeof(SSD_U);
		break;
		case 'V':
		output = SSD_V;
		output_size = sizeof(SSD_V);
		break;
		case 'Y':
		output = SSD_Y;
		output_size = sizeof(SSD_Y);
		break;
		case '-':
		output = SSD_dash;
		output_size = sizeof(SSD_dash);
		break;
		case '$':
		output = SSD_dollar;
		output_size = sizeof(SSD_dollar);
		break;
		case ',':
		output = SSD_comma;
		output_size = sizeof(SSD_comma);
		break;
		case '*':
		output = SSD_asterisk;
		output_size = sizeof(SSD_asterisk);
		break;
		case ':':
		output = SSD_colon;
		output_size = sizeof(SSD_colon);
		break;
		case '.':
		output = SSD_dot;
		output_size = sizeof(SSD_dot);
		break;
		default:
		output = SSD_space;
		output_size = sizeof(SSD_space);
		break;
	}
	
	uint8_t TWI_status = Write_I2C(SSD_ADR, 0x40, (uint8_t *)output, output_size);
	
	return (TWI_status == 4) ? 1 : 0;
}

uint8_t Clear_Display(){
	uint8_t 
		page = 0,
		Clear_Status = 1;
	
	while(1){
		switch (page){
			case 0:
			Clear_Status &= Write_Display(SSD_PAGE0);
			break;
			case 1:
			Clear_Status &= Write_Display(SSD_PAGE1);
			break;
			case 2:
			Clear_Status &= Write_Display(SSD_PAGE2);
			break;
			case 3:
			Clear_Status &= Write_Display(SSD_PAGE3);
			break;
			case 4:
			Clear_Status &= Write_Display(SSD_PAGE4);
			break;
			case 5:
			Clear_Status &= Write_Display(SSD_PAGE5);
			break;
			case 6:
			Clear_Status &= Write_Display(SSD_PAGE6);
			break;
			case 7:
			Clear_Status &= Write_Display(SSD_PAGE7);
			break;
			default:
			break;
		}
		page++;
		if (page > 8){break;}
		else{
			Clear_Status &= Write_Display(0x00);
			Clear_Status &= Write_Display(0x10);
			for (uint8_t j = 0;j<7;j++){
				uint8_t input_data[20] = {0};
				(void)Write_I2C(SSD_ADR, 0x40, input_data, 20);
			}
		}
	}
	return Clear_Status;
}

uint8_t Print_Page(uint8_t page, char *to_print, uint8_t length_to_print){
	uint8_t 
		counter = 0,
		Print_status = 1;
	Print_status &= Write_Display(0x00);
	Print_status &= Write_Display(0x10);
	switch (page){
		case 1:
		Print_status &= Write_Display(SSD_PAGE1);
		break;
		case 2:
		Print_status &= Write_Display(SSD_PAGE2);
		break;
		case 3:
		Print_status &= Write_Display(SSD_PAGE3);
		break;
		case 4:
		Print_status &= Write_Display(SSD_PAGE4);
		break;
		case 5:
		Print_status &= Write_Display(SSD_PAGE5);
		break;
		case 6:
		Print_status &= Write_Display(SSD_PAGE6);
		break;
		case 7:
		Print_status &= Write_Display(SSD_PAGE7);
		break;
		default:
		Print_status &= Write_Display(SSD_PAGE0);
		break;
	}
	while(counter <= length_to_print){
		Print_status &= Write_Character(to_print[counter]);
		Print_status &= Write_Character(' ');
		counter++;
	}
	return Print_status;
}

void Send_Pages(Print_Buffer buffer){
    // Send strings to arduino over SPI2 for it to print on serial monitor
    const int32_t ODR_Hz = 10;
    const Time Sample_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Hz};
    static Time Last_Update = {0};
    static char buffer_c[200] = {0};
//    static char buffer1[] = {'F', 'U', 'C', 'K', 0, 0};
    if ((g_spi2_rdy_flag) && (Compare_And_Update(Current_Time(), Sample_Rate, &Last_Update))){
        uint8_t amount_to_print = sprintf(buffer_c, "%f , %f , %f , %f , %f , %f", buffer.dble[0], buffer.dble[1], buffer.dble[2], buffer.dble[3], buffer.dble[4], buffer.dble[5]) + 2;
        Prepare_SPI2_For_DMA(&CS_ARDUINO_PORT, CS_ARDUINO_PIN, buffer_c);
        Set_DMA_2(&buffer_c[1], amount_to_print);
    }
}