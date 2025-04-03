#include "Ground_Controller.h"

// Barometer (BAR) //
// -> 0 is a write, 1 is a read

unsigned char Setup_Bar(){
	unsigned char BAR_ID = 0;
	unsigned char BAR_status = 2;
	
	BAR_status &= Read_SPI(PORT_BAR,CS_BAR,(BAR_WHO_AM_I|0x80),&BAR_ID,1);
	BAR_status &= Write_SPI(PORT_BAR,CS_BAR,BAR_IF_CTRL,0b00000111); // Disables unused interfaces
	BAR_status &= Write_SPI(PORT_BAR,CS_BAR,BAR_CTRL_REG1,0b01011100); // Sets ODR to 75Hz, enables LPF
	BAR_status &= Write_SPI(PORT_BAR,CS_BAR,BAR_CTRL_REG2,0b00010010); // Enables low noise mode, maximum ODR for this mode is 75 Hz
	
	if (BAR_status != 2){return 0;}
	return 1;
}

unsigned char Read_Bar(float *pressure_altitude){
	static unsigned long pressure_window[BAR_WINDOW_SIZE];
	static unsigned char window_counter = 0;
	unsigned char Read_status = 0;
	unsigned char Data[3] = {0};
	Read_status = Read_SPI(PORT_BAR, CS_BAR, (BAR_DATA_START|0x80), Data, sizeof(Data));
	
	if (Read_status != 2){return 0;}
	
	unsigned long pressure_LSB = (((unsigned long)Data[2])<<16);
	pressure_LSB += (((unsigned int)Data[1])<<8);
	pressure_LSB += Data[0];
	
	pressure_window[window_counter++] = pressure_LSB;
	
	if (window_counter >= BAR_WINDOW_SIZE){
		volatile unsigned long pressure_oversampled = 0;
		for (unsigned char i=0;i<BAR_WINDOW_SIZE;i++){
			pressure_oversampled += pressure_window[i];
		}
		pressure_oversampled >>= 4;
		*pressure_altitude = Height_Bar(pressure_oversampled);
		window_counter = 0;
	}
	
	return 1;
}

float Height_Bar(unsigned long pressure_LSB){
	const float c1 = BAR_Tb/BAR_Lb;
	const float c2 = (-BAR_R*BAR_Lb)/(BAR_g*BAR_M);
	
	float pressure_Pa = ((float)pressure_LSB)*BAR_SENS;
	float height = c1*(pow(pressure_Pa/BAR_Pb,c2)-1.0);
	return height;
}