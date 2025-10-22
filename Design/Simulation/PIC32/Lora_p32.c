#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "Lora_p32.h"
#include "Time_p32.h"
#include "Guidance_p32.h"
#include "Navigation_p32.h"
#include "Controllers_p32.h"
#include "Global_Variables_p32.h"
#include "External_Interface.h"

static Uplink uplink;
static Downlink downlink;
static LORA_Status state;
static Time Last_Update;

void Initialize_LORA(){
    memset(&uplink, 0, sizeof(uplink));
    memset(&downlink, 0, sizeof(downlink));
    memset(&Last_Update, 0, sizeof(Last_Update));
    state = LORA_Standby;
}

void Run_LORA(){
    const int32_t Update_Rate_Hz = 200;
    const Time Update_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/Update_Rate_Hz};
	uint8_t uplink_status;
            
    if (!Compare_And_Update(Current_Time(), Update_Rate, &Last_Update)) return;

    switch (state){
        case LORA_Standby:
            state = LORA_Receiving;
            break;
            
        case LORA_Fail:
            break;

        case LORA_Receiving:				
            uplink_status = Receive_Uplink();
            if (uplink_status){
                state = LORA_Ready_to_Transmit;
            }
            break;

        case LORA_Ready_to_Transmit:
            Send_Downlink();
            state = LORA_Transmitting;
            break;

        case LORA_Transmitting:
            state = LORA_Standby;
            break;
    }
}

uint8_t Receive_Uplink(){
    // Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C_L*CS
	// Underscores are for readability, not part of actual message
	if (!e_uplink_ready) return 0;
    e_uplink_ready = false;
	uint8_t data_available = UPLINK_DATA_SIZE;
	uint8_t amount_to_read = data_available + 3;
	uint8_t uplink_status = 1;
	char buffer_out[255] = {0};
    memcpy(buffer_out, e_uplink_message, UPLINK_SIZE);
	// Keeps track of index in buffer
	uint8_t i = 0;
	// Index in buffer where '$' is, signifies start of message
	int8_t start_index = -1;
	// Index in buffer where '*' is, signifies end of data section of message, beginning of checksum
	int8_t end_index = -1;
	// Populate with uplink message checksum characters
	char Check_Sum[2] = {0};

	while(i != amount_to_read){
		if (buffer_out[i] == '$'){
			start_index = i;
		}
		if ((start_index != -1)&&(buffer_out[i] == '*')){
			end_index = i;
			Check_Sum[0] = buffer_out[++i];
			Check_Sum[1] = buffer_out[++i];
			break;
		}
		i++;
	}

	if ((start_index == -1)||(end_index == -1)){
		uplink_status = 0;
		return 0;
	}
	// Compare checksum in message to calculated checksum
	char checksum_hex[3] = {0};
	Xor_Checksum(buffer_out, UPLINK_DATA_SIZE, start_index+1, checksum_hex);
	// If checksum passes, read uplink
	if ((checksum_hex[0] == Check_Sum[0])&&(checksum_hex[1] == Check_Sum[1])&&uplink_status){
		// Feed the positive coms watchdog
//		g_positive_coms_watchdog = 0;
		downlink.ID[0] = buffer_out[start_index+3];
		downlink.ID[1] = buffer_out[start_index+4];
		// Get desired north/south position
		char inbound_Desired_North[7] = {buffer_out[start_index+5],buffer_out[start_index+6],buffer_out[start_index+7],buffer_out[start_index+8],buffer_out[start_index+9],buffer_out[start_index+10],0};
		double sign = (buffer_out[start_index+11]=='N')?(1.0):(-1.0);
		uplink.Desired_north = sign*atof(inbound_Desired_North);
		// Get desired east/west position
		char inbound_Desired_East[7] = {buffer_out[start_index+12],buffer_out[start_index+13],buffer_out[start_index+14],buffer_out[start_index+15],buffer_out[start_index+16],buffer_out[start_index+17],0};
		sign = (buffer_out[start_index+18]=='E')?(1.0):(-1.0);
		uplink.Desired_east = sign*atof(inbound_Desired_East);
		// Get desired altitude
		char inbound_Desired_Altitude[7] = {buffer_out[start_index+19],buffer_out[start_index+20],buffer_out[start_index+21],buffer_out[start_index+22],buffer_out[start_index+23],buffer_out[start_index+24],0};
		uplink.Desired_altitude = atof(inbound_Desired_Altitude);
        if (fabs(uplink.Desired_altitude) > ALTITUDE_CEILING){
            uplink.Desired_altitude = 0.0;
        }
		// Get base altitude
		char inbound_Base_Altitude[7] = {buffer_out[start_index+25],buffer_out[start_index+26],buffer_out[start_index+27],buffer_out[start_index+28],buffer_out[start_index+29],buffer_out[start_index+30],0};
		uplink.Base_altitude = atof(inbound_Base_Altitude);
		// Get requested drone status
		char Requested_Drone_Status_c[2] = {buffer_out[start_index+31], 0};
		uplink.Drone_status = atoi(Requested_Drone_Status_c);
        if (uplink.Drone_status != g_Flight_Controller_Status){
            Manage_FC_Status(uplink.Drone_status);
        }
        
		return 1;		
	}

	return 0;
}

// Respond to uplink with downlink
void Send_Downlink(){
    // Downlink message format -> $ND_MM_C_T*CS
	// Need to attach TX_offset in LORA data buffer to start of message
	char message[] = {'$', 'N', 'D', downlink.ID[0], downlink.ID[1], g_Flight_Controller_Status, downlink.Tracking_Status, '*', 0, 0, 0};
    char checksum_hex[4] = {0};
    
    // Build checksum
	Xor_Checksum(message, DOWNLINK_DATA_SIZE, 1, checksum_hex);
	message[sizeof(message)-2] = checksum_hex[1];
	message[sizeof(message)-3] = checksum_hex[0];
	
	// Write message into buffer
    memcpy(e_downlink_message, message, DOWNLINK_SIZE);
    e_downlink_ready = true;
}

// State machine to transition drone state
void Manage_FC_Status(FC_Status Desired){
	const char *renum_fcstatus[] = {"Standby", "User_Calibration", "System Calibration", "Ready", "Flying", "Landing", "Crashed"};
    if (Desired > Landing){
        g_Flight_Controller_Status = Standby;
    }
    else{
        g_Flight_Controller_Status = Desired;
    }
    
    switch (g_Flight_Controller_Status){
        case Standby:
            Run_Ground_Filter(true);
            break;
            
        case User_Calibration:
            break;
            
        case System_Calibration:
            break;
            
        case Ready:
            // Save off ground filter states as initial conditions for air filter
            Run_Air_Filter(true);
            Run_Altitude_Filter(true);
            Initialize_Guidance_Machine();
            Initialize_Controllers();
            break;
            
        case Flying:
            break;
            
        case Landing:
            break;
        default:
            break;
                    
    }

	// printf("Flight Controller Status transitioned to %s at %f\n", renum_fcstatus[g_Flight_Controller_Status], Time_fp(Current_Time()));
}

void Xor_Checksum(char *data, uint8_t length, uint8_t start_index, char checksum_hex[3]){
	// checksum_hex must be a null terminated array of 3 characters
	int8_t checksum = data[start_index];
	for (uint8_t i=start_index+1; i<(length+start_index); i++){
		checksum ^= data[i];
	}
	uint8_t converted_length = snprintf(checksum_hex, 3, "%X", checksum);
	if (converted_length == 1){ // Won't add the 0 in automatically if the number is less than 8
		checksum_hex[1] = checksum_hex[0];
		checksum_hex[0] = '0';
	}
}

double Uplink_Altitude(){
    return uplink.Desired_altitude;
}