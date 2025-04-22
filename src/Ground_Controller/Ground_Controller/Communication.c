#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include "Ground_Controller.h"
#include "SSD.h"
#include "LoRa.h"
#include "Utilities.h"
#include "TWI.h"
#include "SPI.h"

// Two Wire Interface (TWI) //
void Setup_TWI(){
	// TWI 0 //
	TWI0_CTRLA |= TWI_SDAHOLD_500NS_gc;
	// BAUD = F_clk/(2*f_scl) - (5 + ((f_clk*TR)/2))
	// -> select baud between 25 and 29 for 350kHz, data sheet gives range of TR of 20-300ns for f_scl <= 400kHz
	TWI0_MBAUD = 25; 
	TWI0_MCTRLA |= TWI_ENABLE_bm | TWI_SMEN_bm | TWI_QCEN_bm; // Enables quick command, smart mode, and master mode
	TWI0_MSTATUS |= TWI_BUSSTATE_IDLE_gc; // Set to idle on startup
	PORTA_PIN2CTRL |= PORT_PULLUPEN_bm;
	PORTA_PIN3CTRL |= PORT_PULLUPEN_bm;
	// TWI 1 //
	TWI1_CTRLA |= TWI_SDAHOLD_500NS_gc;
	TWI1_MBAUD = 25;
	TWI1_MCTRLA |= TWI_ENABLE_bm | TWI_SMEN_bm | TWI_QCEN_bm;
	TWI1_MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	PORTF_PIN2CTRL |= PORT_PULLUPEN_bm;
	PORTF_PIN3CTRL |= PORT_PULLUPEN_bm;
}

unsigned char Write_TWI0(unsigned char Slave_Address, unsigned char Address_Byte, unsigned char *Data, unsigned char Data_Length){
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

unsigned char Write_TWI1(unsigned char Slave_Address, unsigned char Address_Byte, unsigned char *Data, unsigned char Data_Length){
	// Returns 3 if successful, lower numbers indicate stage of failure
	unsigned char i = 0;
	unsigned long timeout = 0;
	TWI1_MADDR = Slave_Address<<1;
	while(!(TWI1_MSTATUS & TWI_WIF_bm)){if (++timeout > TWI_TIMEOUT_THRESHOLD){return 3;}};
	if (TWI1_MSTATUS & TWI_RXACK_bm){return 0;}
	TWI1_MDATA = Address_Byte;
	while(!(TWI1_MSTATUS & TWI_WIF_bm)){if (++timeout > TWI_TIMEOUT_THRESHOLD){return 3;}};
	if (TWI1_MSTATUS & TWI_RXACK_bm){return 1;}
	while (i++ < Data_Length){
		TWI1_MDATA = *Data++;
		while(!(TWI1_MSTATUS & TWI_WIF_bm)){if (++timeout > TWI_TIMEOUT_THRESHOLD){return 3;}};
		if (TWI1_MSTATUS & TWI_RXACK_bm){return 2;}
	}
	TWI1_MCTRLB |= TWI_MCMD_STOP_gc;
	return 4;
}

// Serial Peripheral Interface (SPI) // 
void Setup_SPI(){
	// Set SCK, MOSI, and CS pins as outputs and put CS pins high
	// Here we are using SPI0 on its default pins
	PORTA_DIR |= (1<<CS_BAR) | (1<<MOSI) | (1<<SCK);
	PORTC_DIR |= (1<<CS_LORA);
	PORTA_OUT |= (1<<CS_BAR);
	PORTC_OUT |= (1<<CS_LORA);
	// Enable Master mode
	SPI0_CTRLA |= SPI_MASTER_bm | SPI_ENABLE_bm;
}

unsigned char Read_SPI(char Port, unsigned char Pin, unsigned char Register, unsigned char *Data, unsigned char Data_Length){
	// Returns 2 if successful, 0 if port assignment not valid, and 1 if a timeout occurs while waiting for data
	unsigned char i = 0;
	unsigned long timeout = 0;
	
	if (Port == 'A'){
		PORTA_OUT &= ~(1<<Pin);
	}
	else if (Port == 'C'){
		PORTC_OUT &= ~(1<<Pin);
	}
	else {return 0;}
	SPI0_DATA = Register;
	while (!(SPI0_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	SPI0_INTFLAGS &= ~SPI_IF_bm;
	
	while (i++<Data_Length){
		SPI0_DATA = 0;
		while (!(SPI0_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
		*Data++ = SPI0_DATA;
	}
	
	if (Port == 'A'){
		PORTA_OUT |= (1<<Pin);
	}
	else {
		PORTC_OUT |= (1<<Pin);
	}
	
	return 2;
}

void Read_SPI_c(char Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length){
	unsigned char i = 0;
	
	if (Port == 'A'){
		PORTA_OUT &= ~(1<<Pin);
	}
	else if (Port == 'C'){
		PORTC_OUT &= ~(1<<Pin);
	}
	else return;
	SPI0_DATA = Register;
	while (!(SPI0_INTFLAGS & SPI_IF_bm));
	SPI0_INTFLAGS &= ~SPI_IF_bm;
	
	while (i++<Data_Length){
		SPI0_DATA = 0;
		while (!(SPI0_INTFLAGS & SPI_IF_bm));
		*Data++ = SPI0_DATA;
	}
	
	if (Port == 'A'){
		PORTA_OUT |= (1<<Pin);
	}
	else {
		PORTC_OUT |= (1<<Pin);
	}
}

unsigned char Write_SPI(char Port, unsigned char Pin, unsigned char Register, unsigned char Data){
	// Returns 2 if successful, 0 if port assignment not valid, and 1 if a timeout occurs while waiting for data
	unsigned long timeout = 0;
	
	if (Port == 'A'){
		PORTA_OUT &= ~(1<<Pin);
	}
	else if (Port == 'C'){
		PORTC_OUT &= ~(1<<Pin);
	}
	else {return 0;}
	
	SPI0_DATA = Register;
	while (!(SPI0_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	SPI0_DATA = Data;
	while (!(SPI0_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	
	if (Port == 'A'){
		PORTA_OUT |= (1<<Pin);
	}
	else {
		PORTC_OUT |= (1<<Pin);
	}
	
	return 2;
}

unsigned char Write_SPI_Stream(char Port, unsigned char Pin, unsigned char Register, char *Data, unsigned char Data_Length){
	// Returns 2 if successful, 0 if port assignment not valid, and 1 if a timeout occurs while waiting for data
	unsigned char i = 0;
	unsigned long timeout = 0;
	
	if (Port == 'A'){
		PORTA_OUT &= ~(1<<Pin);
	}
	else if (Port == 'C'){
		PORTC_OUT &= ~(1<<Pin);
	}
	else {return 0;}
	SPI0_DATA = Register;
	while (!(SPI0_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	SPI0_INTFLAGS &= ~SPI_IF_bm;
	
	while (i++<Data_Length){
		SPI0_DATA = *Data++;
		while (!(SPI0_INTFLAGS & SPI_IF_bm)){if (++timeout > SPI_TIMEOUT_THRESHOLD){return 1;}};
	}
	
	if (Port == 'A'){
		PORTA_OUT |= (1<<Pin);
	}
	else {
		PORTC_OUT |= (1<<Pin);
	}
	
	return 2;
}
// Long Range (LORA) // 

volatile unsigned char g_LoRa_Check_Flag = 0;
volatile unsigned char g_LoRa_Uplink_Flag = 0;

unsigned char Setup_LoRa(){
	unsigned char LoRa_status = 2;
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_OP_MODE|0x80),LORA_MODE_SLEEP); // Set to LoRa mode, still in sleep
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_F_MSB|0x80),LORA_FREQ_915_HB); // Set frequency to 915 MHz
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_F_MIDB|0x80),LORA_FREQ_915_MB);
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_F_LSB|0x80),LORA_FREQ_915_LB);
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_IRQ_FLAGS_MASK|0x80),LORA_MASK_TX); // Masks all interrupt flags except TX complete
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_OP_MODE|0x80),LORA_MODE_RXCONTINUOUS); // Set LoRa mode into continuous receive
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_PA_CONFIG|0x80),LORA_PA_20dBm); // Set output gain
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_OCP|0x80),0b00100001); // Set maximum current to 50 mA
	LoRa_status &= Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_PA_RAMP|0x80),0b00000010); // Set PA ramp time to 1ms
	
	if (LoRa_status != 2){return 0;}
	
	return 1;
}

unsigned char Check_For_Message(){
	// Downlink message format -> $ND_ID_C_T*CS
	// Underscores are for readability, not part of actual message
	unsigned char data_available = 0;
	Read_SPI(PORT_LORA,CS_LORA,LORA_REG_RX_N_BYTES,&data_available,1);
	return data_available;
}

Downlink_Reponse_Codes Receive_Downlink(Downlink *inbound, unsigned char ID_index, unsigned char data_available){
	const char ID[12][3] = {
		{'I','e',0},
		{'W','n',0},
		{'o','Y',0},
		{'u','o',0},
		{'l','u',0},
		{'d','r',0},
		{'H','D',0},
		{'a','a',0},
		{'v','d',0},
		{'e','d',0},
		{'B','y',0},
		{'e','.',0},				
	};
	// Downlink message format -> $ND_ID_C_T*CS
	// Underscores are for readability, not part of actual message
	g_LoRa_Check_Flag = 0;
	char buffer[20] = {0};
	unsigned char RX_Adrs = 0;
	(void)Read_SPI(PORT_LORA,CS_LORA,LORA_REG_RX_ADR,&RX_Adrs,1);
	(void)Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_FIFO_ADR_PTR|0x80),RX_Adrs); // Set FIFO ptr to current FIFO RX address
	(void)Read_SPI_c(PORT_LORA,CS_LORA,LORA_REG_FIFO,buffer,data_available);
	(void)Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_FIFO_ADR_PTR|0x80),0x00); // Set FIFO ptr to base RX address

	// Keeps track of index in buffer
	unsigned char i = 0;
	// Index in buffer where '$' is, signifies start of message
	signed char start_index = -1;
	// Index in buffer where '*' is, signifies end of data section of message, beginning of checksum
	signed char end_index = -1;
	// Populate with down link message checksum characters
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

	if ((start_index == -1)||(end_index == -1)) return Bad_format;
	// Compare checksum in message to calculated checksum
	char checksum_hex[3] = {0};
	Xor_Checksum(buffer, 6, start_index+1, checksum_hex);
	// If checksum passes, read downlink
	if ((checksum_hex[0] == Check_Sum[0])&&(checksum_hex[1] == Check_Sum[1])){
		char inbound_ID[3] = {buffer[3], buffer[4], 0};
		if (strcmp(inbound_ID, ID[ID_index]) == 0){
			inbound->Calibration_Status = buffer[5];
			inbound->Tracking_Status = buffer[6];
			return Good_response;
		}
		return Bad_ID;
	}
	return Bad_checksum;
}

unsigned char Send_Uplink(Uplink *outbound, Downlink_Reponse_Codes *Downlink_Status){
	// Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C*CS
	const char ID[12][2] = {
		{'I','e'},
		{'W','n'},
		{'o','Y'},
		{'u','o'},
		{'l','u'},
		{'d','r'},
		{'H','D'},
		{'a','a'},
		{'v','d'},
		{'e','d'},
		{'B','y'},
		{'e','.'},
	};
	static unsigned char ID_index;
	// If the check flag wasn't cleared, than we never got a response
	if (g_LoRa_Check_Flag == 1) *Downlink_Status = No_response;
	// Cycle message ID
	ID_index = (ID_index<11)?(ID_index+1):(0);
	g_LoRa_Uplink_Flag = 0;
	// TX data must be written to LoRa while it is in standby
	(void)Write_SPI(PORT_LORA, CS_LORA, (LORA_REG_OP_MODE|0x80), LORA_MODE_SLEEP);
	// Convert desired positions and current base altitude to characters for transmission
	char buffer[5][7];
	sprintf(buffer[0], "%06.2f", fabs(outbound->Desired_north));
	char north_south = (outbound->Desired_north >= 0)?('N'):('S');
	sprintf(buffer[1], "%06.2f", fabs(outbound->Desired_east));
	char east_west = (outbound->Desired_east >= 0)?('E'):('W');
	sprintf(buffer[2], "%06.2f", fabs(outbound->Desired_altitude));
	sprintf(buffer[3], "%06.2f", fabs(outbound->Pressure_altitude));
	sprintf(buffer[4], "%d", outbound->Drone_status);
	// Build up link message
	char message[] = {'$', 'N', 'D', ID[ID_index][0], ID[ID_index][1],
		 buffer[0][0], buffer[0][1], buffer[0][2], buffer[0][3], buffer[0][4], buffer[0][5], north_south,
		 buffer[1][0], buffer[1][1], buffer[1][2], buffer[1][3], buffer[1][4], buffer[1][5], east_west,
		 buffer[2][0], buffer[2][1], buffer[2][2], buffer[2][3], buffer[2][4], buffer[2][5],
		 buffer[3][0], buffer[3][1], buffer[3][2], buffer[3][3], buffer[3][4], buffer[3][5], buffer[4][0],
		 '*', 0, 0, 0};
	// Build checksum
	char checksum_hex[3] = {0};
	Xor_Checksum(message, (sizeof(message)-5), 1, checksum_hex);
	message[sizeof(message)-2] = checksum_hex[1];
	message[sizeof(message)-3] = checksum_hex[0];
	// Set FIFO pointer to TX base address, and write message
	unsigned char TX_base_adr;
	(void)Read_SPI(PORT_LORA,CS_LORA,LORA_REG_TX_BASE_ADR,&TX_base_adr,1);
	(void)Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_FIFO_ADR_PTR|0x80),TX_base_adr);
	(void)Write_SPI(PORT_LORA,CS_LORA,(LORA_REG_PAYLOAD_LENGTH|0x80),sizeof(message)-1);
	(void)Write_SPI_Stream(PORT_LORA, CS_LORA, (LORA_REG_FIFO|0x80), message, sizeof(message)-1);
	(void)Write_SPI(PORT_LORA, CS_LORA, (LORA_REG_OP_MODE|0x80), LORA_MODE_TX);
	unsigned char LoRa_TX_Status = 0;
	while(1){
		Read_SPI(PORT_LORA, CS_LORA, LORA_REG_IRQ_FLAGS, &LoRa_TX_Status, 1);
		if (LoRa_TX_Status == LORA_IRQ_TX_DONE){
			break;
		}
		Delay(1000);
	}
	// Transmission complete, put LoRa back into receive continuous mode
	(void)Write_SPI(PORT_LORA, CS_LORA, (LORA_REG_OP_MODE|0x80), LORA_MODE_STDBY);
	Delay(1000);
	(void)Write_SPI(PORT_LORA, CS_LORA, (LORA_REG_OP_MODE|0x80), LORA_MODE_RXCONTINUOUS);
	unsigned char output = ID_index;
	
	g_LoRa_Check_Flag = 1;
	
	return output;
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
				up_link->Drone_status = Standby;
				break;
			case Ready:
				up_link->Drone_status = Flying;
				break;
			case Flying:
				up_link->Drone_status = Landing;
				break;
			case Landing:
			// Drone can only move out of landing status to ready internally
				up_link->Drone_status = Flying;
				break;
		}
	}
}

// SOLOMON SYSTECH DRIVER (SSD) 1306 CODE

volatile unsigned char g_Print_Flag = 0;

unsigned char Setup_SSD(unsigned char Display){
	unsigned char Setup_status = 1;
	// From data sheet - order of software tasks to initialize display
	//Setup_status &= Write_Display(SSD_DISPLAY_OFF);
	// 1. Set MUX ratio -> 0xA8, 0x3F
	Setup_status &= Write_Display_Double(SSD_MULTIPLEX_RATIO, 0x3F, Display);
	// 2. Set Display Offset -> 0xD3, 0x00
	Setup_status &= Write_Display_Double(SSD_DISPLAY_OFFSET, 0x00, Display);
	// 3. Set Display Start Line -> 0x40
	Setup_status &= Write_Display(SSD_DISPLAY_START_LINE, Display);
	// 4. Set Segment re-map -> 0xA0
	Setup_status &= Write_Display(SSD_SEGMENT_REMAP, Display);
	// 5. Set COM Output Scan Direction -> 0xC0
	Setup_status &= Write_Display(SSD_COM_OUTPUT_SCAN_DIRECTION, Display);
	// 6. Set COM Pins hardware configuration -> 0xDA, 02
	Setup_status &= Write_Display_Double(SSD_COM_PINS_CONFIGURATION, 0x02, Display);
	// 7. Set Contrast Control -> 0x81, 0x7F
	Setup_status &= Write_Display_Double(SSD_CONTRAST_CONTROL, 0x7F, Display);
	// 8. Disable Entire Display On -> 0xA4
	Setup_status &= Write_Display(SSD_ENTIRE_DISPLAY_RAM, Display);
	// 9. Set Normal Display -> 0xA6
	Setup_status &= Write_Display(SSD_NORMAL_DISPLAY, Display);
	// 10. Set Oscillator Frequency -> 0xD5, 0x80
	Setup_status &= Write_Display_Double(SSD_OSC_FREQUENCY, 0x80, Display);
	// 11. Enable charge pump regulator -> 0x8D, 0x14
	Setup_status &= Write_Display_Double(SSD_CHARGE_PUMP, 0x14, Display);
	// 12. Display On -> 0xAF
	Setup_status &= Write_Display(SSD_DISPLAY_ON, Display);

	Setup_status &= Clear_Display(Display);
	
	return Setup_status;
}

inline unsigned char Write_Display(unsigned char Data_Byte, unsigned char Display){
	unsigned char TWI_status = 0;
	if (Display == 0){
		TWI_status = Write_TWI0(SSD_ADR, 0x80, &Data_Byte, 1);
	}
	else if (Display == 1){
		TWI_status = Write_TWI1(SSD_ADR, 0x80, &Data_Byte, 1);
	}
	
	return (TWI_status == 4) ? 1 : 0;
}

inline unsigned char Write_Display_Double(unsigned char Address_Byte, unsigned char Data_Byte, unsigned char Display){
	unsigned char input_data[2] = {Address_Byte, Data_Byte};
	unsigned char TWI_status = 0;
	if (Display == 0){
		TWI_status = Write_TWI0(SSD_ADR, 0x00, input_data, 2);
	}
	else if (Display == 1){
		TWI_status = Write_TWI1(SSD_ADR, 0x00, input_data, 2);
	}
	
	return (TWI_status == 4) ? 1 : 0;
}

unsigned char Write_Character(char Character_to_write, unsigned char Display){
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
	
	if (Display == 0){
		(void)Write_TWI0(SSD_ADR, 0x40, (unsigned char *)output, output_size);
	}
	else if (Display == 1){
		(void)Write_TWI1(SSD_ADR, 0x40, (unsigned char *)output, output_size);
	}
	
	return output_size;
}

unsigned char Clear_Display(unsigned char Display){
	unsigned char page = 0;
	unsigned char Clear_Status = 1;
	
	while(1){
		switch (page){
			case 0:
			Clear_Status &= Write_Display(SSD_PAGE0, Display);
			break;
			case 1:
			Clear_Status &= Write_Display(SSD_PAGE1, Display);
			break;
			case 2:
			Clear_Status &= Write_Display(SSD_PAGE2, Display);
			break;
			case 3:
			Clear_Status &= Write_Display(SSD_PAGE3, Display);
			break;
			case 4:
			Clear_Status &= Write_Display(SSD_PAGE4, Display);
			break;
			case 5:
			Clear_Status &= Write_Display(SSD_PAGE5, Display);
			break;
			case 6:
			Clear_Status &= Write_Display(SSD_PAGE6, Display);
			break;
			case 7:
			Clear_Status &= Write_Display(SSD_PAGE7, Display);
			break;
			default:
			break;
		}
		page++;
		if (page > 8){break;}
		else{
			Clear_Status &= Write_Display(0x00, Display);
			Clear_Status &= Write_Display(0x10, Display);
			for (unsigned char j = 0;j<7;j++){
				unsigned char input_data[20] = {0};
				if (Display == 0){
					(void)Write_TWI0(SSD_ADR, 0x40, input_data, 20);
				}
				else if (Display == 1){
					(void)Write_TWI1(SSD_ADR, 0x40, input_data, 20);
				}
			}
		}
	}
	return Clear_Status;
}

unsigned char Print_Page(unsigned char page, char *to_print, unsigned char length_to_print, unsigned char Display){
	unsigned char counter = 0;
	unsigned char Print_status = 1;
	unsigned char printed_length = 0;
	Print_status &= Write_Display(0x00, Display);
	Print_status &= Write_Display(0x10, Display);
	switch (page){
		case 1:
		Print_status &= Write_Display(SSD_PAGE1, Display);
		break;
		case 2:
		Print_status &= Write_Display(SSD_PAGE2, Display);
		break;
		case 3:
		Print_status &= Write_Display(SSD_PAGE3, Display);
		break;
		case 4:
		Print_status &= Write_Display(SSD_PAGE4, Display);
		break;
		case 5:
		Print_status &= Write_Display(SSD_PAGE5, Display);
		break;
		case 6:
		Print_status &= Write_Display(SSD_PAGE6, Display);
		break;
		case 7:
		Print_status &= Write_Display(SSD_PAGE7, Display);
		break;
		default:
		Print_status &= Write_Display(SSD_PAGE0, Display);
		break;
	}
	while(counter <= length_to_print){
		printed_length += Write_Character(to_print[counter], Display);
		printed_length += Write_Character(' ', Display);
		counter++;
	}
	for (unsigned char i = printed_length; i < SSD_PAGE_LENGTH; i+=3){
		Print_status &= Write_Character(' ', Display);
	}
	return Print_status;
}

void Print_Displays(Dial *D_h, Dial *D_n, Dial *D_e, Uplink *up_link, Downlink_Reponse_Codes Downlink_Status){
	char *Dl_S_renums[5] = {"NO RESPONSE","BAD FORMAT","BAD ID","BAD CHECKSUM","GOOD RESPONSE"};
	char *Dr_S_renums[5] = {"STANDBY","CALIBRATING","READY","FLYING","LANDING"};
	char buffer[3][20];
	g_Print_Flag = 0;
	
	// Printing on display 0 (right)
	unsigned char length_to_print = snprintf(buffer[0], sizeof(buffer[0]), "HEIGHT: %1.2f", D_h->output);
	Print_Page(0, buffer[0], length_to_print, 0);
	length_to_print = snprintf(buffer[1], sizeof(buffer[1]), "EAST: %1.2f", D_e->output);
	Print_Page(1, buffer[1], length_to_print, 0);
	length_to_print = snprintf(buffer[2], sizeof(buffer[2]), "NORTH: %1.2f", D_n->output);
	Print_Page(2, buffer[2], length_to_print, 0);
	
	// Printing on display 1 (left)
	Print_Page(0, Dr_S_renums[up_link->Drone_status], strlen(Dr_S_renums[up_link->Drone_status]), 1);
	Print_Page(1, Dl_S_renums[Downlink_Status], strlen(Dl_S_renums[Downlink_Status]), 1);
}