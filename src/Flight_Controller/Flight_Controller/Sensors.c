// Includes - generic
#include <avr/io.h>
#include <avr/xmega.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Bar.h"
#include "IMU.h"
#include "Mag.h"
#include "GPS.h"
#include "Observer.h"
#include "FC_Types.h"
#include "SPI.h"
// BAROMETER CODE
// Bar -> 0 is a write, 1 is a read

volatile unsigned char g_BAR_Read_Flag = 0;

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

void Calibrate_Bar(States *Drone, Calibration_Data *cal_data, float base_altitude){
	if (abs(Drone->pressure_altitude - base_altitude) < 1.0){
		cal_data->base_altitude = base_altitude;
		cal_data->bar_cal_status = 1;
	}
}

void Read_Bar(States *Drone, Calibration_Data *cal_data){
	static unsigned long pressure_window[BAR_WINDOW_SIZE];
	static unsigned char window_counter;
	
	g_BAR_Read_Flag = 0;
	
	unsigned char Read_status = 0;
	unsigned char Data[3] = {0};
	Read_status = Read_SPI(PORT_BAR, CS_BAR, (BAR_DATA_START|0x80), Data, sizeof(Data));
	
	if (Read_status != 2){return;}
	
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
		Drone->Position_NED[2] = -(Height_Bar(pressure_oversampled) - cal_data->base_altitude);
		window_counter = 0;
	}
	
}

float Height_Bar(unsigned long pressure_LSB){
	const float c1 = BAR_TB/BAR_LB;
	const float c2 = (-BAR_R*BAR_LB)/(BAR_G*BAR_M);
	
	float pressure_Pa = ((float)pressure_LSB)*BAR_SENS;
	float height = c1*(pow(pressure_Pa/BAR_PB,c2)-1.0);
	return height;
}
// IMU CODE
// IMU -> 0 is a write, 1 is a read

volatile unsigned char g_IMU_Read_Flag = 0;

unsigned char Setup_IMU(){
	// Configure IMU
	unsigned char IMU_status = 2;
	unsigned char IMU_ID = 0;
	
	IMU_status &= Read_SPI(PORT_IMU, CS_IMU, (IMU_WHO_AM_I|0x80), &IMU_ID, 1);
	IMU_status &= Write_SPI(PORT_IMU, CS_IMU, IMU_CTRL1_XL, 0b01010000); // Sets Accelerometer ODR to 208 Hz, range to +-2g
	IMU_status &= Write_SPI(PORT_IMU, CS_IMU, IMU_CTRL2_G, 0b01010100); // Sets Gyro ODR to 208 Hz, range to +-500dps
	IMU_status &= Write_SPI(PORT_IMU, CS_IMU, IMU_CTRL8_XL, 0b11001000); // Sets Accelerometer LPF to ODR/9, low noise
	
	if(IMU_status != 2){return 0;}
	return 1;
}

void Calibrate_IMU(States *Drone, Calibration_Data *cal_data){
	// Flip positive directions on Gyro x and z axis to align with Forward-Right-Down coordinate system (aligns with NED when not rotated)
	unsigned long w_magnitude = abs(Drone->w_xyz_LSB[0]) + abs(Drone->w_xyz_LSB[1]) + abs(Drone->w_xyz_LSB[2]);
	
	if (w_magnitude <= W_CAL_LIMIT){
		cal_data->w_bias[0] = -Drone->w_xyz_LSB[0];
		cal_data->w_bias[1] = Drone->w_xyz_LSB[1];
		cal_data->w_bias[2] = -Drone->w_xyz_LSB[2];
	}
	
	cal_data->imu_cal_status = 1;
}

void Read_IMU(States *Drone, Calibration_Data *cal_data){
	static signed int a_xyz_window[3][IMU_WINDOW_SIZE];
	static signed int w_xyz_window[3][GYRO_WINDOW_SIZE];
	static unsigned char window_counter_a;
	static unsigned char window_counter_g;
	unsigned char Read_status = 0;
	
	g_IMU_Read_Flag = 0;
	
	unsigned char Data[12] = {0};
	Read_status = Read_SPI(PORT_IMU, CS_IMU, (IMU_DATA_START|0x80), Data, sizeof(Data));
	if (Read_status != 2) return;
	
	for (unsigned char i=0;i<3;i++){
		a_xyz_window[i][window_counter_a] = (((signed int)Data[2*i+7])<<8) + Data[2*i+6];
		w_xyz_window[i][window_counter_g] = (((signed int)Data[2*i+1])<<8) + Data[2*i];
	}
	window_counter_a++;
	window_counter_g++;
	
	if (window_counter_g >= GYRO_WINDOW_SIZE){
		window_counter_g = 0;
		for (unsigned char i=0;i<3;i++){
			Drone->w_xyz_LSB[i] = w_xyz_window[i][0] + w_xyz_window[i][1];
			Drone->w_xyz_LSB[i] >>= 1;
		}
		// Flip positive directions on Gyro x and z axis to align with Forward-Right-Down coordinate system (aligns with NED when not rotated)
		signed int w_diff[3] = {-Drone->w_xyz_LSB[0]-cal_data->w_bias[0], Drone->w_xyz_LSB[1]-cal_data->w_bias[1], -Drone->w_xyz_LSB[2]-cal_data->w_bias[2]};
		for (unsigned char i=0;i<3;i++){
			Drone->w[i] = ((float)w_diff[i])*GYRO_SENS*D2R;
		}
	}
	
	if (window_counter_a >= IMU_WINDOW_SIZE){
		window_counter_a = 0;
		signed long a_xyz_FIR[3] = {0};
		for (unsigned char i=0;i<3;i++){
			for (unsigned char j=0;j<IMU_WINDOW_SIZE;j++){
				a_xyz_FIR[i] += a_xyz_window[i][j];
			}
			a_xyz_FIR[i] >>= 3;
		}
		// Flip positive directions on Accelerometer y axis to align with Forward-Right-Down coordinate system (aligns with NED when not rotated)
		Drone->g_vec[0] = ((float)a_xyz_FIR[0])*ACCEL_SENS;
		Drone->g_vec[1] = ((float)-a_xyz_FIR[1])*ACCEL_SENS;
		Drone->g_vec[2] = ((float)a_xyz_FIR[2])*ACCEL_SENS;
	}
	
}

// MAGNETOMETER CODE
// Mag -> 0 is a write, 1 is a read

volatile unsigned char g_MAG_Read_Flag = 0;

unsigned char  Setup_Mag(){
	unsigned char MAG_status = 2;
	unsigned char Mag_ID = 0;
	
	MAG_status &= Write_SPI(PORT_MAG, CS_MAG, MAG_CFG_REG_C, 0b00110100); // Enables 4 wire SPI, disables I2C
	MAG_status &= Read_SPI(PORT_MAG, CS_MAG, (MAG_WHO_AM_I|0x80), &Mag_ID, 1);
	MAG_status &= Write_SPI(PORT_MAG, CS_MAG, MAG_CFG_REG_A, 0b10001100); // Sets continuous mode, 100 Hz ODR, temp compensation enabled
	MAG_status &= Write_SPI(PORT_MAG, CS_MAG, MAG_CFG_REG_B, 0b00000001); // Enables LPF
	
	if (MAG_status != 2){return 0;}
	return 1;
}

void Calibrate_Mag(States *Drone, Calibration_Data *cal_data){
	for (unsigned char i = 0; i < 3; i++){
		if (Drone->m_xyz_LSB[i] > cal_data->m_max[i]){
			cal_data->m_max[i] = Drone->m_xyz_LSB[i];
			if (abs(cal_data->m_max[i])<abs(cal_data->m_min[i])){
				cal_data->hard_iron[i] = cal_data->m_min[i]-cal_data->m_max[i];
			}
			else{
				cal_data->hard_iron[i] = cal_data->m_max[i]-cal_data->m_min[i];
			}
			//cal_data->hard_iron[i] >>= 1;
		}
		else if (Drone->m_xyz_LSB[i] < cal_data->m_min[i]){
			cal_data->m_min[i] = Drone->m_xyz_LSB[i];
			if (abs(cal_data->m_max[i])<abs(cal_data->m_min[i])){
				cal_data->hard_iron[i] = cal_data->m_min[i]-cal_data->m_max[i];
			}
			else{
				cal_data->hard_iron[i] = cal_data->m_max[i]-cal_data->m_min[i];
			}
			//cal_data->hard_iron[i] >>= 1;
		}
	}

}

void Read_Mag(States *Drone, Calibration_Data *cal_data){
	static signed int m_xyz_window[3][MAG_WINDOW_SIZE];
	static unsigned char window_counter;
	unsigned char Read_status = 0;
	
	g_MAG_Read_Flag = 0;
	unsigned char Data[6] = {0};
	Read_status = Read_SPI(PORT_MAG, CS_MAG, (MAG_DATA_START|0x80), Data, 6);
	if (Read_status != 2) return;
	
	for (unsigned char i=0;i<3;i++){
		m_xyz_window[i][window_counter] = (((signed int)Data[2*i+1])<<8) + Data[2*i];
	}
	
	window_counter++;
	if (window_counter >= MAG_WINDOW_SIZE){
		window_counter = 0;
		for (unsigned char i=0;i<3;i++){
			Drone->m_xyz_LSB[i] = 0;
			for (unsigned char j=0;j<MAG_WINDOW_SIZE;j++){
				Drone->m_xyz_LSB[i] += m_xyz_window[i][j];
			}
			Drone->m_xyz_LSB[i] >>= 2;
		}

		Drone->m_vec[0] = ((float)(Drone->m_xyz_LSB[1] - cal_data->hard_iron[1]))/((float)cal_data->hard_iron[1]);
		Drone->m_vec[1] = -((float)(Drone->m_xyz_LSB[0]- cal_data->hard_iron[0]))/((float)cal_data->hard_iron[0]);
		Drone->m_vec[2] = ((float)(Drone->m_xyz_LSB[2] - cal_data->hard_iron[2]))/((float)cal_data->hard_iron[2]);

	}
	
}

// GPS CODE
static volatile char g_GPS_Data[256];
static volatile unsigned char g_GPS_Data_Index;
volatile unsigned char g_GPS_Read_Flag = 0;

unsigned char Setup_GPS(){
	USART3_BAUD = 2500; // Corresponds to 38400 baud rate
	PORTB_DIR |= (1<<0);
	USART3_CTRLA |= USART_RXCIE_bm;
	USART3_CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
	//USART3_DBGCTRL |= 1;
	Delay(10);
	if (USART3_RXDATAH & USART_FERR_bm){
		USART3_BAUD = 208; // Corresponds to 460800 baud rate
		USART3_RXDATAH = USART_FERR_bm;
	}
	else{
		char Increase_Baud[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x08,0x07,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x0C,0xBC};
		USART_Transmit(Increase_Baud, sizeof(Increase_Baud));
		USART3_BAUD = 208; // Corresponds to 460800 baud rate
	}
	Delay(100000);
	
	// Disable all the messages we don't want
	char Disable_DTM[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x69};
	USART_Transmit(Disable_DTM, sizeof(Disable_DTM));
	char Disable_GBQ[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x44,0x00,0x00,0x00,0x00,0x00,0x00,0x43,0xFF};
	USART_Transmit(Disable_GBQ, sizeof(Disable_GBQ));
	char Disable_GBS[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x62};
	USART_Transmit(Disable_GBS, sizeof(Disable_GBS));
	char Disable_GGA[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x23};
	USART_Transmit(Disable_GGA, sizeof(Disable_GGA));
	char Disable_GLL[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A};
	USART_Transmit(Disable_GLL, sizeof(Disable_GLL));
	char Disable_GLQ[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x42,0xF8};
	USART_Transmit(Disable_GLQ, sizeof(Disable_GLQ));
	char Disable_GNQ[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0xF1};
	USART_Transmit(Disable_GNQ, sizeof(Disable_GNQ));
	char Disable_GNS[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x0D,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x7E};
	USART_Transmit(Disable_GNS, sizeof(Disable_GNS));
	char Disable_GPQ[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xE3};
	USART_Transmit(Disable_GPQ, sizeof(Disable_GPQ));
	char Disable_GRS[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x4D};
	USART_Transmit(Disable_GRS, sizeof(Disable_GRS));
	char Disable_GSA[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x31};
	USART_Transmit(Disable_GSA, sizeof(Disable_GSA));
	char Disable_GST[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x54};
	USART_Transmit(Disable_GST, sizeof(Disable_GST));
	char Disable_GSV[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38};
	USART_Transmit(Disable_GSV, sizeof(Disable_GSV));
	char Disable_THS[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x85};
	USART_Transmit(Disable_THS, sizeof(Disable_THS));
	char Disable_TXT[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0xEA};
	USART_Transmit(Disable_TXT, sizeof(Disable_TXT));
	char Disable_VLW[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x8C};
	USART_Transmit(Disable_VLW, sizeof(Disable_VLW));
	char Disable_VTG[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46};
	USART_Transmit(Disable_VTG, sizeof(Disable_VTG));
	char Disable_ZDA[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x5B};
	USART_Transmit(Disable_ZDA, sizeof(Disable_ZDA));

	
	// Increase the update rate to 8Hz
	char Enable_UTC_8Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x7D, 0x00, 0x01, 0x00, 0x00, 0x00, 0x92, 0xA6};
	USART_Transmit(Enable_UTC_8Hz, sizeof(Enable_UTC_8Hz));
	char Enable_GPS_8Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x7D, 0x00, 0x01, 0x00, 0x01, 0x00, 0x93, 0xA8};
	USART_Transmit(Enable_GPS_8Hz, sizeof(Enable_GPS_8Hz));
	char Enable_GLO_8Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x7D, 0x00, 0x01, 0x00, 0x02, 0x00, 0x94, 0xAA};
	USART_Transmit(Enable_GLO_8Hz, sizeof(Enable_GLO_8Hz));
	char Enable_BDS_8Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x7D, 0x00, 0x01, 0x00, 0x03, 0x00, 0x95, 0xAC};
	USART_Transmit(Enable_BDS_8Hz, sizeof(Enable_BDS_8Hz));
	char Enable_GAL_8Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x7D, 0x00, 0x01, 0x00, 0x04, 0x00, 0x96, 0xAE};
	USART_Transmit(Enable_GAL_8Hz, sizeof(Enable_GAL_8Hz));

	return (USART3_RXDATAH & USART_FERR_bm) ? 0 : 1;
}

void Calibrate_GPS(States *Drone, Calibration_Data *cal_data){
	if ((Drone->Longitude != -1)&&(Drone->Latitude != -1)){
			const float a = 6378137.0; // Earth Equatorial Radius (m)
			const float b = 6356752.3142; // Earth Polar Radius (m)
			const float e = (pow(a,2)-pow(b,2))/pow(a,2); // Eccentricity
			
			float Latitude_rad = (((float)Drone->Latitude)/6000000)*D2R;
			float Longitude_rad = (((float)Drone->Longitude)/6000000)*D2R;
			float N_phi = a/sqrt(1.0 - (pow(e,2)*pow(sinf(Latitude_rad),2))); // Prime vertical radius (m)
			
			cal_data->Reference_Position_ecef[0] = (N_phi - Drone->Position_NED[2])*cosf(Latitude_rad)*cosf(Longitude_rad);
			cal_data->Reference_Position_ecef[1] = (N_phi - Drone->Position_NED[2])*cosf(Latitude_rad)*sinf(Longitude_rad);
			cal_data->Reference_Position_ecef[2] = (((1.0-pow(e,2))*N_phi) - Drone->Position_NED[2])*sinf(Latitude_rad);
			cal_data->gps_cal_status = 1;
	}
}


void Read_GPS(States *Drone, Calibration_Data *cal_data){
	static signed long 
		Latitude_window[GPS_WINDOW_SIZE],
		Longitude_window[GPS_WINDOW_SIZE];
    static char 
		GPS_Position_Status,
		GPS_Position_Mode;
	static unsigned char 
		window_counter;
	unsigned char 
		i = 0,
		j = 0;
	signed char 
		start_index = -1,
		end_index = -1,
		comma_indices[13] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
	char 
		GPS_Data[100] = {0},
		Check_Sum[2] = {0};
			
	g_GPS_Read_Flag = 0;
	
	while(i != g_GPS_Data_Index){
		if (g_GPS_Data[i] == 36){start_index = i;}
		else if ((g_GPS_Data[i] == 42)&&(start_index != -1)){
			end_index = i;
			Check_Sum[0] = g_GPS_Data[++i];
			Check_Sum[1] = g_GPS_Data[++i];
			break;
		}
		else if ((g_GPS_Data[i] == 44)&&(start_index != -1)){comma_indices[j++] = i;}
		if (start_index != -1){GPS_Data[i] = g_GPS_Data[i];}
		i++;
	}
	g_GPS_Data_Index = 0;
	if ((start_index == -1)||(end_index == -1)) return;
	
	signed char checksum = GPS_Data[start_index+1];
	for (unsigned char k=start_index+2;k<=end_index;k++){
		checksum ^= GPS_Data[k];
	}
	char checksum_hex[3] = {0};
	unsigned char converted_length = snprintf(checksum_hex, sizeof(checksum_hex), "%X", checksum);
	if (converted_length == 1){ // Won't add the 0 in automatically if the number is less than 8
		checksum_hex[1] = checksum_hex[0];
		checksum_hex[0] = 48;
	}
	
	if ((checksum_hex[0] == Check_Sum[0])&&(checksum_hex[1] == Check_Sum[1])){
		// If the checksum signifies that the message is valid, then we go through parsing it
		// NMEA RMC information comes in the following order:
		// -> Name, time, status, latitude, NS, longitude, EW, speed, course over ground, date,
		// magnetic variation, magnetic variation EW, position mode, navigation status
		if (comma_indices[2] - comma_indices[1] - 1){
			GPS_Position_Status = GPS_Data[comma_indices[1]+1];
		}
		if (comma_indices[12] - comma_indices[11] - 1){
			GPS_Position_Mode = GPS_Data[comma_indices[11]+1];
		}
		if ((GPS_Position_Status == 'A')&&(GPS_Position_Mode == 'A')){
			if (comma_indices[3]-comma_indices[2] - 1){
				char Latitude_degrees[] = {GPS_Data[comma_indices[2]+1], GPS_Data[comma_indices[2]+2], '\0'};
				char Latitude_minutes_integer[] = {GPS_Data[comma_indices[2]+3], GPS_Data[comma_indices[2]+4], '\0'};
				char Latitude_minutes_decimal[] = {GPS_Data[comma_indices[2]+6], GPS_Data[comma_indices[2]+7], GPS_Data[comma_indices[2]+8], GPS_Data[comma_indices[2]+9], GPS_Data[comma_indices[2]+10], '\0'};
				Latitude_window[window_counter] = ((atol(Latitude_degrees)*60)+atol(Latitude_minutes_integer))*100000 + atol(Latitude_minutes_decimal);
			}
			if (comma_indices[4] - comma_indices[3] - 1){
				Latitude_window[window_counter] *= (GPS_Data[comma_indices[3]+1] == 'N') ? 1 : -1;
			}
			if (comma_indices[5]-comma_indices[4] - 1){
				char Longitude_degrees[] = {GPS_Data[comma_indices[4]+1], GPS_Data[comma_indices[4]+2], GPS_Data[comma_indices[4]+3], '\0'};
				char Longitude_minutes_integer[] = {GPS_Data[comma_indices[4]+4], GPS_Data[comma_indices[4]+5], '\0'};
				char Longitude_minutes_decimal[] = {GPS_Data[comma_indices[4]+7], GPS_Data[comma_indices[4]+8], GPS_Data[comma_indices[4]+9], GPS_Data[comma_indices[4]+10], GPS_Data[comma_indices[4]+11], '\0'};
				Longitude_window[window_counter] = ((atol(Longitude_degrees)*60)+atol(Longitude_minutes_integer))*100000 + atol(Longitude_minutes_decimal);
			}
			if (comma_indices[6] - comma_indices[5] - 1){
				Longitude_window[window_counter] *= (GPS_Data[comma_indices[5]+1] == 'W') ? -1 : 1;
			}
			if (comma_indices[7] - comma_indices[6] - 1){
				unsigned char k = comma_indices[7] - comma_indices[6];
				char Speed[6] = {0};
				char *ptr;
				for (unsigned char index=0;index<k;index++){
					Speed[index] = GPS_Data[comma_indices[6]+1+index];
				}
				Drone->Speed_over_ground = Drone->Speed_over_ground*0.9 + strtod(Speed, &ptr)*0.1;
			}
			if (comma_indices[8] - comma_indices[7] - 1){
				unsigned char k = comma_indices[8] - comma_indices[7];
				char Course[6];
				char *ptr;
				for (unsigned char index=0;index<k;index++){
					Course[index] = GPS_Data[comma_indices[7]+1+index];
				}
				Drone->Course_over_ground = strtod(Course, &ptr);
			}
			
			if (++window_counter >= GPS_WINDOW_SIZE){
				window_counter = 0;
				signed long long Latitude_oversampled = 0;
				signed long long Longitude_oversampled = 0;
				for (unsigned char k=0;k<GPS_WINDOW_SIZE;k++){
					Latitude_oversampled += Latitude_window[k];
					Longitude_oversampled += Longitude_window[k];
				}
				Latitude_oversampled >>= 2;
				Longitude_oversampled >>= 2;
				Drone->Longitude = Longitude_oversampled;
				Drone->Latitude = Latitude_oversampled;
				LLA_to_NED(Latitude_oversampled, Longitude_oversampled, Drone->Position_NED, cal_data->Reference_Position_ecef);
			}
		}
	}
}

void LLA_to_NED(signed long Latitude, signed long Longitude, float Position_NED[3], float Reference_Position_ecef[3]){
	// First convert LLA -> ECEF using WGS 84
	const float 
		a = 6378137.0, // Earth Equatorial Radius (m)
		b = 6356752.3142, // Earth Polar Radius (m)
		e = (pow(a,2)-pow(b,2))/pow(a,2), // Eccentricity
		c1 = 0.9,
		c2 = 1.0-c1;
	float Position_ecef[3];
	
	float Latitude_rad = (((float)Latitude)/6000000)*D2R;
	float Longitude_rad = (((float)Longitude)/6000000)*D2R;
	float N_phi = a/sqrt(1.0 - (pow(e,2)*pow(sinf(Latitude_rad),2))); // Prime vertical radius (m)
	
	Position_ecef[0] = (N_phi - Position_NED[2])*cosf(Latitude_rad)*cosf(Longitude_rad);
	Position_ecef[1] = (N_phi - Position_NED[2])*cosf(Latitude_rad)*sinf(Longitude_rad);
	Position_ecef[2] = (((1.0-pow(e,2))*N_phi) - Position_NED[2])*sinf(Latitude_rad);
	
	float dx_ecef[3];
	dx_ecef[0] = Position_ecef[0] - Reference_Position_ecef[0];
	dx_ecef[1] = Position_ecef[1] - Reference_Position_ecef[1];
	dx_ecef[2] = Position_ecef[2] - Reference_Position_ecef[2];
	
	float P_North= -sinf(Latitude_rad)*cosf(Longitude_rad)*dx_ecef[0] - sinf(Latitude_rad)*sinf(Longitude_rad)*dx_ecef[1] + cosf(Latitude_rad)*dx_ecef[2];
	float P_East = -sinf(Longitude_rad)*dx_ecef[0] + cosf(Longitude_rad)*dx_ecef[1];
	Position_NED[0] = Position_NED[0]*c1 + P_North*c2;
	Position_NED[1] = Position_NED[1]*c1 + P_East*c2;
	
}

void USART_Transmit(char* Message, unsigned char length){
	for (unsigned char i=0;i<length;i++){
		USART3_TXDATAL = *Message++;
		while(!(USART3_STATUS & USART_TXCIF_bm));
		USART3_STATUS = USART_TXCIF_bm;
	}
	Delay(1000);
}

ISR(USART3_RXC_vect){
	while(USART3_STATUS & USART_RXCIF_bm){
		char temp = USART3_RXDATAL;
		if (temp == 10){g_GPS_Read_Flag = 1;}
		g_GPS_Data[g_GPS_Data_Index++] = temp;
	}
}

// OBSERVER CODE

volatile unsigned char g_Attitude_Observer_Run_Flag = 0;

void Observer(States *Drone){
	const float 
		L = 0.05, // Observer gain, increasing the gain increases the trust on the model(gyro),
	// and decreasing it increases the trust on the measurement (accelerometer and magnetometer)
		dt = 0.04, // Time between integrations
		Gimbal_Lock_Check_Angle = 5.0*D2R;
	// Measure
	
	g_Attitude_Observer_Run_Flag = 0;

	float phi_m = atan2f(Drone->g_vec[1], Drone->g_vec[2]);
	if (isnan(phi_m)){
		phi_m = Drone->Euler[0];
	}
	float theta_m = atan2f(-Drone->g_vec[0], sqrt(pow(Drone->g_vec[1],2) + pow(Drone->g_vec[2],2)));
	if (isnan(theta_m)){
		theta_m = Drone->Euler[1];
	}
	float mag_x_NED = cosf(Drone->Euler[1])*Drone->m_vec[0] + sinf(Drone->Euler[0])*sinf(Drone->Euler[1])*Drone->m_vec[1] + cosf(Drone->Euler[0])*sinf(Drone->Euler[1])*Drone->m_vec[2];
	float mag_y_NED = cosf(Drone->Euler[0])*Drone->m_vec[1] - sinf(Drone->Euler[0])*Drone->m_vec[2];
	float psi_m = -atan2f(mag_y_NED, mag_x_NED);
	if (isnan(psi_m)){
		psi_m = Drone->Euler[2];
	}
	if (psi_m <= 0){
		psi_m += 2.0*M_PI;
	}
	
	// Predict
	float phi_hat = Drone->Euler[0];
	float theta_hat = Drone->Euler[1];
	float psi_hat = Drone->Euler[2];
	if (abs(abs(Drone->Euler[1]) - M_PI_2) > Gimbal_Lock_Check_Angle){
		phi_hat += (Drone->w[0] + sinf(Drone->Euler[0])*tanf(Drone->Euler[1])*Drone->w[1] + cosf(Drone->Euler[0])*tanf(Drone->Euler[1])*Drone->w[2])*dt;
		theta_hat += (cosf(Drone->Euler[0])*Drone->w[1] - sinf(Drone->Euler[0])*Drone->w[2])*dt;
		psi_hat += ((sinf(Drone->Euler[0])/cosf(Drone->Euler[1]))*Drone->w[1] + (cosf(Drone->Euler[0])/cosf(Drone->Euler[1]))*Drone->w[2])*dt;
	}
	// Update
	Drone->Euler[0] = phi_hat + L*(phi_m-phi_hat);
	Drone->Euler[1] = theta_hat + L*(theta_m-theta_hat);
	// Prevent yaw angle discontinuity at 2pi - 0 to cause the filter to slowly cycle between them
	Drone->Euler[2] = (abs(psi_m-psi_hat)>M_PI)?(psi_m):(psi_hat + L*(psi_m-psi_hat));
}