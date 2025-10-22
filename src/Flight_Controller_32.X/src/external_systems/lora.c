#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "lora.h"
#include "pins.h"
#include "spi.h"
#include "time.h"
#include "navigation.h"
#include "guidance.h"
#include "controllers.h"
#include "global_variables.h"
#include "system_types.h"

static Uplink uplink;
static Downlink downlink;
static LORA_Status state;
static Time Last_Update;

void Initialize_LORA_Machine(){
    memset(&uplink, 0, sizeof(uplink));
    memset(&downlink, 0, sizeof(downlink));
    memset(&Last_Update, 0, sizeof(Last_Update));
    state = Setup_LoRa();
}

// Manage LORA state
void Run_LORA(){
    const int32_t Update_Rate_Hz = 200;
    const Time Update_Rate = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/Update_Rate_Hz};
	uint8_t rx_timeout[4] = {LORA_SETRX, 0xFF, 0xFF, 0xFF};
	uint8_t lora_irq_status_out[5] = {0};
    uint8_t lora_irq_status_in[5] = {LORA_GET_IRQ_STATUS, 0, 0, 0, 0};
    uint8_t lora_irq_clear[3] = {LORA_CLEAR_IRQ_STATUS, 0, LORA_TX_DONE_IRQ};
	uint8_t uplink_status;
    uint8_t dummy_out[10];
            
    if (!g_spi1_rdy_flag || !Compare_And_Update(Current_Time(), Update_Rate, &Last_Update)) return;

    switch (state){
        case LORA_Standby:
            // Put LORA in RXContinuous mode
            SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, rx_timeout, dummy_out, sizeof(rx_timeout));
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
            // LORA busy pin goes low when the chip has reached a stable state
            if (!(LORA_BUSY_PORT & (1<<LORA_BUSY_PIN))){
                // LORA IRQ status will indicate if the transmission is complete yet
                SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, lora_irq_status_in, lora_irq_status_out, sizeof(lora_irq_status_in));
                if (lora_irq_status_out[3] & LORA_TX_DONE_IRQ){
                    state = LORA_Standby;
                    SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, lora_irq_clear, dummy_out, sizeof(lora_irq_clear));
                }
            }
            break;
    }

}

LORA_Status Setup_LoRa(){
    // Take CS pin low, wait for busy to go low to indicate LORA modem is out of sleep mode
    LOWER_PIN(CS_LORA_PORT, CS_LORA_PIN);
    LORA_Delay(1000000);
    RAISE_PIN(CS_LORA_PORT, CS_LORA_PIN);
    
    // Put LORA in standby mode
    uint8_t lora_mode[2] = {LORA_SETSTANDBY, 0};
    uint8_t dummy_out[10] = {0};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, lora_mode, dummy_out, sizeof(lora_mode));
	LORA_Delay(500000);
    
	float frequency_sensitivity = (pow(2,25))/(32000000.0);
	uint32_t frequency = (uint32_t)(910000000.0*frequency_sensitivity); // Corresponds to 910MHz -> frequency = freq_Hz * (2^25/32e6)
	uint8_t freq_data[5] = {LORA_SET_RF_FREQ, (uint8_t)(frequency>>24), (uint8_t)(frequency>>16), (uint8_t)(frequency>>8), (uint8_t)(frequency)};
	// Set frequency to 910MHz	
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, freq_data, dummy_out, sizeof(freq_data)); 
	LORA_Delay(LORA_STANDARD_DELAY);
    
	// Set packet type to LORA
    uint8_t packet_type_set[2] = {LORA_SET_PACKET, 0x01};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, packet_type_set, dummy_out, sizeof(packet_type_set));
	LORA_Delay(LORA_STANDARD_DELAY);
    
	// Confirm packet type was set to LORA
    uint8_t dummy_in[3] = {0x11, 0, 0};
    uint8_t packet_type_read[3] = {0};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, dummy_in, packet_type_read, sizeof(packet_type_read)); 
	if (packet_type_read[2] != 1) return LORA_Fail;
    
	// Set power to 18dBm and ramp time to 3.4 ms
	uint8_t tx_params[3] = {LORA_SET_TX_PARAMS, 0x12, 0x07};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, tx_params, dummy_out, sizeof(tx_params));
	LORA_Delay(LORA_STANDARD_DELAY);
    
	// Set PA duty cycle and HP max
	uint8_t pa_config[5] = {LORA_SETPA, 0x02, 0x03, 0x00, 0x01};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, pa_config, dummy_out, sizeof(pa_config));
	LORA_Delay(LORA_STANDARD_DELAY);
    
	// Change LORA sync word to match up with 127x series
	uint8_t sync_word_lsb[4] = {LORA_WRITE_REGISTER, (uint8_t)(0x0741>>8), (uint8_t)(0x0741), (uint8_t)(LORA_SYNC_WORD)};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, sync_word_lsb, dummy_out, sizeof(sync_word_lsb));
	LORA_Delay(LORA_STANDARD_DELAY);
    
	uint8_t sync_word_msb[4] = {LORA_WRITE_REGISTER, (uint8_t)(0x0740>>8), (uint8_t)(0x0740), (uint8_t)(LORA_SYNC_WORD>>8)};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, sync_word_msb, dummy_out, sizeof(sync_word_msb));
	LORA_Delay(100000);
    
	// Set preamble length
	uint8_t packet_params[7] = {LORA_SET_PACKET_PARAMS, 0, 8, 0, 12, 0, 0};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, packet_params, dummy_out, sizeof(packet_params));
	LORA_Delay(LORA_STANDARD_DELAY);
    
	// Set LORA modulation parameters, Spreading factor: 11, 500kHz BW, CR48
	uint8_t mod_params[5] = {LORA_SET_MOD_PARAMS, 0x0B, 0x06, 0x04, 0x01};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, mod_params, dummy_out, sizeof(mod_params));
	LORA_Delay(LORA_STANDARD_DELAY);
    
	// Set LORA IRQ parameters
	uint8_t irq_params[3] = {LORA_SET_DIO_IRQ_PARAMS, 0, 0b00000011};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, irq_params, dummy_out, sizeof(irq_params));
	LORA_Delay(LORA_STANDARD_DELAY);
    
	// Set Buffer base addresses
	uint8_t base_addresses[3] = {LORA_SET_BUFFER_BASE_ADR, TX_BASE_ADR, RX_BASE_ADR};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, base_addresses, dummy_out, sizeof(base_addresses));
	LORA_Delay(LORA_STANDARD_DELAY);
    
	// Set LORA in FS Mode
    lora_mode[0] = LORA_SETFS;
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, lora_mode, dummy_out, sizeof(lora_mode));
    
    return LORA_Standby;

}

// Receive and parse uplink
uint8_t Receive_Uplink(){
    // Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C_L*CS
	// Underscores are for readability, not part of actual message
	uint8_t rx_offset = 0;
	
	uint8_t data_available = Check_For_Message(&rx_offset);
	if (data_available < UPLINK_SIZE) return 0;
	
	uint8_t amount_to_read = data_available + 3;
	uint8_t uplink_status = 1;
	uint8_t buffer_in[255] = {0};
	char buffer_out[255] = {0};
	buffer_in[0] = LORA_READ_BUFFER;
	buffer_in[1] = rx_offset;
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, buffer_in, (uint8_t *)buffer_out, amount_to_read);
    uint8_t lora_mode[2] = {LORA_SETSTANDBY, 1};
    uint8_t dummy_out[2] = {0};
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, lora_mode, dummy_out, sizeof(lora_mode));
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
	char message[] = {TX_BASE_ADR, '$', 'N', 'D', downlink.ID[0], downlink.ID[1], g_Flight_Controller_Status, downlink.Tracking_Status, '*', 0, 0, 0};
    char checksum_hex[4] = {0};
    uint8_t data_in[sizeof(message)+1];
    uint8_t dummy_out[sizeof(message)+1];
    uint32_t timeout = 0;
	uint8_t tx_timeout[4] = {LORA_SETTX, (uint8_t)(timeout>>16), (uint8_t)(timeout>>8), (uint8_t)timeout};
    
    // Build checksum
	Xor_Checksum(message, INTERCHANGE_DATA_SIZE, 2, checksum_hex);
	message[sizeof(message)-2] = checksum_hex[1];
	message[sizeof(message)-3] = checksum_hex[0];
	
	// Write message into buffer
    data_in[0] = LORA_WRITE_BUFFER;
    memcpy(&data_in[1], message, sizeof(message));
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, data_in, dummy_out, sizeof(data_in));
    
	// Start ramping and transmission
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, tx_timeout, dummy_out, sizeof(tx_timeout));
}

// State machine to transition drone state
void Manage_FC_Status(FC_Status Desired){
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
                    
    }
}

// Get payload length in LoRa FIFO
uint8_t Check_For_Message(uint8_t *rx_offset){
    // Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C*CS
	// Underscores are for readability, not part of actual message
	uint8_t lora_irq_status_out[5] = {0};
    uint8_t lora_irq_status_in[5] = {LORA_GET_IRQ_STATUS, 0, 0, 0, 0};
    uint8_t lora_rx_status_in[4] = {LORA_GET_RX_BUFFER_STATUS, 0, 0, 0};
    uint8_t lora_rx_status_out[4] = {0};
    uint8_t payload_length;
    uint8_t lora_irq_clear[3] = {LORA_CLEAR_IRQ_STATUS, 0, LORA_RX_DONE_IRQ};
    uint8_t dummy_out[3] = {0};
    
	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, lora_irq_status_in, lora_irq_status_out, sizeof(lora_irq_status_out));
	if (!(lora_irq_status_out[3] & LORA_RX_DONE_IRQ)) return 0;
    

	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, lora_rx_status_in, lora_rx_status_out, sizeof(lora_rx_status_in));
	payload_length = lora_rx_status_out[2];
    *rx_offset = lora_rx_status_out[3];

	SPI_transfer(&CS_LORA_PORT, CS_LORA_PIN, lora_irq_clear, dummy_out, sizeof(lora_irq_clear));
	
	return payload_length;
}

// Delay and check LORA busy pin, will go low when LORA is ready
void LORA_Delay(uint32_t length){
    Delay(length);
	while(LORA_BUSY_PORT & (1<<LORA_BUSY_PIN));
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
