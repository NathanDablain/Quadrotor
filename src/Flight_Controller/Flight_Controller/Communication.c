#include <avr/io.h>
#include <avr/xmega.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "SPI.h"
#include "TWI.h"
#include "LoRa.h"
#include "Utilities.h"
#include "SSD.h"
#include "FC_Types.h"

// SERIAL PERIPHERAL INTERFACE (SPI) CODE
void Setup_SPI(){
	// Set SCK, MOSI, and CS pins as outputs and put CS pins high
	// Here we are using SPI1 on its default pins
	PORTA_DIR |= (1<<CS_LORA) | (1<<CS_IMU) | (1<<CS_BAR);
	PORTB_DIR |= (1<<CS_DGW) | (1<<CS_MAG);
	PORTC_DIR |= (1<<MOSI) | (1<<SCK);
	PORTA_OUT |= (1<<CS_LORA) | (1<<CS_IMU) | (1<<CS_BAR);
	PORTB_OUT |= (1<<CS_DGW) | (1<<CS_MAG);
	// Enable Master mode
	SPI1_CTRLA |= SPI_MASTER_bm | SPI_ENABLE_bm;
}

void Read_SPI(volatile unsigned char *Port, unsigned char Pin, unsigned char Register, unsigned char *Data, unsigned int Data_Length){
	unsigned int i = 0;
	
	*Port &= ~(1<<Pin);
		
	SPI1_DATA = Register;
	while (!(SPI1_INTFLAGS & SPI_IF_bm));
	SPI1_INTFLAGS &= ~SPI_IF_bm;
	
	while (i++<Data_Length){
		SPI1_DATA = 0;
		while (!(SPI1_INTFLAGS & SPI_IF_bm));
		*Data++ = SPI1_DATA;
	}
	
	*Port |= (1<<Pin);
}

void Read_SPI_c(volatile unsigned char *Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length){
	unsigned char i = 0;
	
	*Port &= ~(1<<Pin);

	SPI1_DATA = Register;
	while (!(SPI1_INTFLAGS & SPI_IF_bm));
	SPI1_INTFLAGS &= ~SPI_IF_bm;
	
	while (i++<Data_Length){
		SPI1_DATA = 0;
		while (!(SPI1_INTFLAGS & SPI_IF_bm));
		*Data++ = SPI1_DATA;
	}
	
	*Port |= (1<<Pin);
}

unsigned char Write_SPI(volatile unsigned char *Port, unsigned char Pin, unsigned char Register, unsigned char Data){
	// Returns 2 if successful, 0 if port assignment not valid, and 1 if a timeout occurs while waiting for data
	unsigned long timeout = 0;
	
	*Port &= ~(1<<Pin);
	
	SPI1_DATA = Register;
	while (!(SPI1_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	SPI1_DATA = Data;
	while (!(SPI1_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	
	*Port |= (1<<Pin);
	
	return 2;
}

unsigned char Write_SPI_Stream(volatile unsigned char *Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length){
	// Returns 2 if successful, 0 if port assignment not valid, and 1 if a timeout occurs while waiting for data
	unsigned char i = 0;
	unsigned long timeout = 0;
	
	*Port &= ~(1<<Pin);

	SPI1_DATA = Register;
	while (!(SPI1_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	SPI1_INTFLAGS &= ~SPI_IF_bm;
	
	while (i++<Data_Length){
		SPI1_DATA = *Data++;
		while (!(SPI1_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	}

	*Port |= (1<<Pin);
	
	return 2;
}

// TWO WIRE INTERFACE (TWI) CODE
void Setup_TWI(){
	TWI0_CTRLA |= TWI_SDAHOLD_500NS_gc;
	// BAUD = F_clk/(2*f_scl) - (5 + ((f_clk*TR)/2))
	// -> select baud between 25 and 29 for 350kHz, data sheet gives range of TR of 20-300ns for f_scl <= 400kHz
	TWI0_MBAUD = 25; //109;
	TWI0_MCTRLA |= TWI_ENABLE_bm | TWI_SMEN_bm | TWI_QCEN_bm; // Enables quick command, smart mode, and master mode
	TWI0_MSTATUS |= TWI_BUSSTATE_IDLE_gc; // Set to idle on startup
	PORTA_PIN2CTRL |= PORT_PULLUPEN_bm;
	PORTA_PIN3CTRL |= PORT_PULLUPEN_bm;
	//TWI0_DBGCTRL |= TWI_DBGRUN_bm;
}

unsigned char Write_TWI(unsigned char Slave_Address, unsigned char Address_Byte, unsigned char *Data, unsigned char Data_Length){
	// Returns 3 if successful, lower numbers indicate stage of failure
	unsigned char i = 0;
	unsigned long timeout = 0;
	TWI0_MADDR = Slave_Address<<1;
	while(!(TWI0_MSTATUS & TWI_WIF_bm)){if (++timeout > TWI_TIMEOUT_THRESHOLD){return 3;}};
	if (TWI0_MSTATUS & TWI_RXACK_bm){return 0;}
	TWI0_MDATA = Address_Byte;
	while(!(TWI0_MSTATUS & TWI_WIF_bm)){if (++timeout > TWI_TIMEOUT_THRESHOLD){return 3;}};
	if (TWI0_MSTATUS & TWI_RXACK_bm){return 1;}
	while (i++ < Data_Length){
		TWI0_MDATA = *Data++;
		while(!(TWI0_MSTATUS & TWI_WIF_bm)){if (++timeout > TWI_TIMEOUT_THRESHOLD){return 3;}};
		if (TWI0_MSTATUS & TWI_RXACK_bm){return 2;}
	}
	TWI0_MCTRLB |= TWI_MCMD_STOP_gc;
	return 4;
}

// LONG RANGE (LORA) CODE
volatile unsigned char g_LoRa_Check_Flag = 0;
volatile unsigned char g_LoRa_Send_Flag = 0;

unsigned char Setup_LoRa(){
	unsigned char LoRa_status = 2;
	
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_OP_MODE|0x80),LORA_MODE_SLEEP); // Set LoRa mode, still in sleep
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_F_MSB|0x80),LORA_FREQ_915_HB); // Set frequency to 915 MHz
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_F_MIDB|0x80),LORA_FREQ_915_MB);
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_F_LSB|0x80),LORA_FREQ_915_LB);
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_IRQ_FLAGS_MASK|0x80),LORA_MASK_TX); // Masks all interrupt flags except TX complete
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_OP_MODE|0x80),LORA_MODE_RXCONTINUOUS); // Set to continuously receive
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_PA_CONFIG|0x80),LORA_PA_20dBm); // Set power to 20 dBm
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_OCP|0x80),0b00100001); // Set maximum current to 50 mA
	LoRa_status &= Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_PA_RAMP|0x80),0b00000010); // Set PA ramp time to 1ms
	
	return (LoRa_status == 2) ? 1 : 0;
}

unsigned char Check_For_Message(){
	// Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C*CS
	// Underscores are for readability, not part of actual message
	unsigned char data_available = 0;
	Read_SPI(&PORTA_OUT,CS_LORA,LORA_REG_RX_N_BYTES,&data_available,1);
	return data_available;
}

void Receive_Uplink(Uplink *inbound, Downlink *outbound, FC_Status *Flight_Controller_Status){
	// Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C*CS
	// Underscores are for readability, not part of actual message
	g_LoRa_Check_Flag = 0;
	(void)Write_SPI(&PORTA_OUT, CS_LORA, (LORA_REG_OP_MODE|0x80), LORA_MODE_RXCONTINUOUS);

	unsigned char data_available = Check_For_Message();
	if (data_available < UPLINK_SIZE) return;
	
	unsigned char uplink_status = 1;
	char buffer[50] = {0};
	unsigned char RX_Adrs = 0;
	(void)Read_SPI(&PORTA_OUT,CS_LORA,LORA_REG_RX_ADR,&RX_Adrs,1);
	(void)Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_FIFO_ADR_PTR|0x80),RX_Adrs); // Set FIFO ptr to current FIFO RX address
	(void)Read_SPI_c(&PORTA_OUT,CS_LORA,LORA_REG_FIFO,buffer,data_available);

	// Keeps track of index in buffer
	unsigned char i = 0;
	// Index in buffer where '$' is, signifies start of message
	signed char start_index = -1;
	// Index in buffer where '*' is, signifies end of data section of message, beginning of checksum
	signed char end_index = -1;
	// Populate with uplink message checksum characters
	char Check_Sum[2] = {0};

	while(i != data_available){
		if (buffer[i] == '$'){
			start_index = i;
		}
		if ((start_index != -1)&&(buffer[i] == '*')){
			end_index = i;
			Check_Sum[0] = buffer[++i];
			Check_Sum[1] = buffer[++i];
			break;
		}
		i++;
	}

	if ((start_index == -1)||(end_index == -1)) uplink_status = 0;
	// Compare checksum in message to calculated checksum
	char checksum_hex[3] = {0};
	Xor_Checksum(buffer, (end_index-start_index), start_index+1, checksum_hex);
	// If checksum passes, read uplink
	if ((checksum_hex[0] == Check_Sum[0])&&(checksum_hex[1] == Check_Sum[1])&&uplink_status){
		outbound->ID[0] = buffer[start_index+3];
		outbound->ID[1] = buffer[start_index+4];
		// Get desired north/south position
		char inbound_Desired_North[7] = {buffer[start_index+5],buffer[start_index+6],buffer[start_index+7],buffer[start_index+8],buffer[start_index+9],buffer[start_index+10],0};
		float sign = (buffer[start_index+11]=='N')?(1.0):(-1.0);
		inbound->Desired_north = sign*atof(inbound_Desired_North);
		// Get desired east/west position
		char inbound_Desired_East[7] = {buffer[start_index+12],buffer[start_index+13],buffer[start_index+14],buffer[start_index+15],buffer[start_index+16],buffer[start_index+17],0};
		sign = (buffer[start_index+18]=='E')?(1.0):(-1.0);
		inbound->Desired_east = sign*atof(inbound_Desired_East);
		// Get desired altitude
		char inbound_Desired_Altitude[7] = {buffer[start_index+19],buffer[start_index+20],buffer[start_index+21],buffer[start_index+22],buffer[start_index+23],buffer[start_index+24],0};
		inbound->Desired_altitude = atof(inbound_Desired_Altitude);
		// Get base altitude
		char inbound_Base_Altitude[7] = {buffer[start_index+25],buffer[start_index+26],buffer[start_index+27],buffer[start_index+28],buffer[start_index+29],buffer[start_index+30],0};
		inbound->Base_altitude = atof(inbound_Base_Altitude);
		// Get requested drone status
		char Requested_Drone_Status_c[2] = {buffer[start_index+31], 0};
		inbound->Drone_status = atoi(Requested_Drone_Status_c);
		*Flight_Controller_Status = Manage_FC_Status(inbound->Drone_status, *Flight_Controller_Status);
		outbound->Flight_Controller_Status = *Flight_Controller_Status;
		
		g_LoRa_Send_Flag = 1;
	}
}

void Send_Downlink(Downlink *outbound){
	// Downlink message format -> $ND_MM_C_T*CS
	g_LoRa_Send_Flag = 0;
	(void)Write_SPI(&PORTA_OUT, CS_LORA, (LORA_REG_OP_MODE|0x80), LORA_MODE_SLEEP);

	// Build downlink message
	char message[] = {'$', 'N', 'D', outbound->ID[0], outbound->ID[1], outbound->Flight_Controller_Status, outbound->Tracking_Status, '*', 0, 0, 0};
	// Build checksum
	char checksum_hex[3] = {0};
	Xor_Checksum(message, (sizeof(message)-5), 1, checksum_hex);
	message[sizeof(message)-2] = checksum_hex[1];
	message[sizeof(message)-3] = checksum_hex[0];
	
	// Set FIFO pointer to TX base address, and write message
	unsigned char TX_base_adr = 0;
	(void)Read_SPI(&PORTA_OUT,CS_LORA,LORA_REG_TX_ADR,&TX_base_adr,1);
	(void)Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_FIFO_ADR_PTR|0x80),TX_base_adr);
	(void)Write_SPI(&PORTA_OUT,CS_LORA,(LORA_REG_PAYLOAD_LENGTH|0x80),sizeof(message)-1);
	(void)Write_SPI_Stream(&PORTA_OUT, CS_LORA, (LORA_REG_FIFO|0x80), message, sizeof(message)-1);
	(void)Write_SPI(&PORTA_OUT, CS_LORA, (LORA_REG_OP_MODE|0x80), LORA_MODE_TX);
	unsigned char LoRa_TX_Status = 0;
	while(1){
		Read_SPI(&PORTA_OUT, CS_LORA, LORA_REG_IRQ_FLAGS, &LoRa_TX_Status, 1);
		if (LoRa_TX_Status == LORA_IRQ_TX_DONE){
			break;
		}
	}
	
}

FC_Status Manage_FC_Status(FC_Status Desired, FC_Status Current){
	switch (Desired){
		case Standby:
			switch (Current){
				case Standby:
					return Current;
				case Calibrating:
					return Desired;
				case Ready:
					return Desired;
				case Flying:
					return Landing;
				case Landing:
					return Current;
				default:
					return Current;
			}
			break;
		case Calibrating:
			switch (Current){
				case Standby:
					return Desired;
				case Calibrating:
					return Current;
				case Ready:
					return Current;
				case Flying:
					return Landing;
				case Landing:
					return Current;
				default:
					return Current;
			}
			break;
		case Ready:
			switch (Current){
				case Standby:
					return Current;
				case Calibrating:
					return Current;
				case Ready:
					return Current;
				case Flying:
					return Landing;
				case Landing:
					return Current;
				default:
					return Current;
			}
			break;
		case Flying:
			switch (Current){
				case Standby:
					return Current;
				case Calibrating:
					return Current;
				case Ready:
					return Desired;
				case Flying:
					return Current;
				case Landing:
					return Desired;
				default:
					return Current;
			}
			break;
		case Landing:
			switch (Current){
				case Standby:
					return Current;
				case Calibrating:
					return Current;
				case Ready:
					return Current;
				case Flying:
					return Desired;
				case Landing:
					return Current;
				default:
					return Current;
			}
			break;	
		default: 
			return Current;
	}
}

// SOLOMON SYSTECH DRIVER (SSD) 1306 CODE

volatile unsigned char g_Print_Flag = 0;

unsigned char Setup_SSD(){
	unsigned char Setup_status = 1;
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

inline unsigned char Write_Display(unsigned char Data_Byte){
	unsigned char TWI_status = Write_TWI(SSD_ADR, 0x80, &Data_Byte, 1);
	
	return (TWI_status == 4) ? 1 : 0;
}

inline unsigned char Write_Display_Double(unsigned char Address_Byte, unsigned char Data_Byte){
	unsigned char input_data[2] = {Address_Byte, Data_Byte};
	unsigned char TWI_status = Write_TWI(SSD_ADR, 0x00, input_data, 2);
	
	return (TWI_status == 4) ? 1 : 0;
}

unsigned char Write_Character(char Character_to_write){
	// Always leave bottom bit blank, bits order from bottom (7) to top (0)
	const unsigned char SSD_space[3] = {0x00, 0x00, 0x00};
	const unsigned char SSD_dot[3] = {0x00, 0b01000000, 0x00};
	const unsigned char SSD_dash[3] = {0b00001000, 0b00001000, 0b00001000};
	const unsigned char SSD_comma[3] = {0b00100000, 0b01100000, 0b00000000};
	const unsigned char SSD_asterisk[3] = {0b00000010, 0b00000111, 0b00000010};
	const unsigned char SSD_dollar[5] = {0b01001111, 0b01001001, 0b011111111, 0b01001001, 0b01111001};
	const unsigned char SSD_colon[2] = {0b00100100, 0b00100100};
	const unsigned char SSD_0[4] = {0b00111110, 0b01000001, 0b01000001, 0b00111110};
	const unsigned char SSD_1[3] = {0b01000010, 0b01111111, 0b01000000};
	const unsigned char SSD_2[4] = {0b01111001, 0b01001001, 0b01001001, 0b01001111};
	const unsigned char SSD_3[5] = {0b01000001, 0b01001001, 0b01001001, 0b01010101, 0b00110110};
	const unsigned char SSD_4[5] = {0b00010000, 0b00011000, 0b00010100, 0b00010010, 0b01111111};
	const unsigned char SSD_5[4] = {0b01001111, 0b01001001, 0b01001001, 0b01111001};
	const unsigned char SSD_6[5] = {0b00011100, 0b00101010, 0b01001001, 0b00101001, 0b00010000};
	const unsigned char SSD_7[6] = {0b01000001, 0b00100001, 0b00010001, 0b00001001, 0b00000101, 0b00000011};
	const unsigned char SSD_8[5] = {0b00010100, 0b00101010, 0b01001001, 0b00101010, 0b00010100};
	const unsigned char SSD_9[5] = {0b01000110, 0b00101010, 0b00011001, 0b00001010, 0b00000100};
	const unsigned char SSD_A[5] = {0b01111100, 0b00001010, 0b00001001, 0b00001010, 0b01111100};
	const unsigned char SSD_B[5] = {0b01111111, 0b01001001, 0b01001001, 0b01011101, 0b00100010};
	const unsigned char SSD_C[4] = {0b00011100, 0b00100010, 0b01000001, 0b00100010};
	const unsigned char SSD_D[4] = {0b01111111, 0b01000001, 0b00100010, 0b00011100};
	const unsigned char SSD_E[4] = {0b01111111, 0b01001001, 0b01001001, 0b01001001};
	const unsigned char SSD_F[4] = {0b01111111, 0b00010001, 0b00010001, 0b00000001};
	const unsigned char SSD_G[5] = {0b00011100, 0b00100010, 0b01010001, 0b01010001, 0b01110010};
	// SSD_H refers to the header file
	const unsigned char SSD_h[5] = {0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111};
	const unsigned char SSD_I[5] = {0b01000001, 0b01000001, 0b01111111, 0b01000001, 0b01000001};
	const unsigned char SSD_J[5] = {0b00100001, 0b01000001, 0b00100001, 0b00011111, 0b00000001};
	const unsigned char SSD_K[5] = {0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001};
	const unsigned char SSD_L[4] = {0b01111111, 0b01000000, 0b01000000, 0b01000000};
	const unsigned char SSD_M[5] = {0b01111111, 0b00000010, 0b00000100, 0b00000010, 0b01111111};
	const unsigned char SSD_N[7] = {0b01111111, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01111111};
	const unsigned char SSD_O[4] = {0b00111110, 0b01000001, 0b01000001, 0b00111110};
	const unsigned char SSD_P[4] = {0b01111111, 0b00001001, 0b00001001, 0b00000110};
	const unsigned char SSD_Q[6] = {0b00011100, 0b00100010, 0b01000001, 0b01000101, 0b00100010, 0b00011101};
	const unsigned char SSD_R[5] = {0b01111111, 0b00001001, 0b00011001, 0b00100110, 0b01000000};
	// S copies from 5
	const unsigned char SSD_T[5] = {0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001};
	const unsigned char SSD_U[5] = {0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111};
	const unsigned char SSD_V[11] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010, 0b00000001};
	//const unsigned char SSD_W[];
	//const unsigned char SSD_X[];
	const unsigned char SSD_Y[5] = {0b00000001, 0b00000010, 0b01111100, 0b00000010, 0b00000001};
	//const unsigned char SSD_Z[];
	
	const unsigned char *output;
	unsigned char output_size;
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
	
	unsigned char TWI_status = Write_TWI(SSD_ADR, 0x40, (unsigned char *)output, output_size);
	
	return (TWI_status == 4) ? 1 : 0;
}

unsigned char Clear_Display(){
	unsigned char 
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
			for (unsigned char j = 0;j<7;j++){
				unsigned char input_data[20] = {0};
				(void)Write_TWI(SSD_ADR, 0x40, input_data, 20);
			}
		}
	}
	return Clear_Status;
}

unsigned char Print_Page(unsigned char page, char *to_print, unsigned char length_to_print){
	unsigned char 
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

void Print_Output(States *Drone, Calibration_Data *cal_data, Uplink *uplink){
	g_Print_Flag = 0;
	char buffer[4][20] = {0};
	unsigned char length_to_print = snprintf(buffer[0], sizeof(buffer[0]), "%2.1f , %2.1f, %2.1f", Drone->Euler[0], Drone->Euler[1], Drone->Euler[2]);
	Print_Page(0, buffer[0], length_to_print);
	length_to_print = snprintf(buffer[1], sizeof(buffer[1]), "%d, %d, %d", cal_data->w_bias[0], cal_data->w_bias[1], cal_data->w_bias[2]);
	Print_Page(1, buffer[1], length_to_print);
	length_to_print = snprintf(buffer[2], sizeof(buffer[2]), "%d, %d, %d", cal_data->hard_iron[0], cal_data->hard_iron[1], cal_data->hard_iron[2]);
	Print_Page(2, buffer[2], length_to_print);
	length_to_print = snprintf(buffer[3], sizeof(buffer[3]), "%3.1f,%3.1f,%3.1f", cal_data->altitude_bias, uplink->Base_altitude, -Drone->Position_NED[2]);
	Print_Page(3, buffer[3], length_to_print);
}