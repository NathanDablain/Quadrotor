#ifndef LORA_H
#define	LORA_H

#include "system_types.h"
#include <stdint.h>

// Protocol for 7th bit when using SPI interface:
// -> 1 is a write, 0 is a read

// Downlink message format -> $ND_MM_C_T*CS
// Downlink message size -> 10, '_' are for readability, not part of message
// Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C*CS
// Uplink message size -> 35, '_' are for readability, not part of message
// Downlink MM should match most recent Uplink MM

// Ground Controller sends an uplink at 1Hz, Flight controller should respond with a downlink upon receiving the uplink
// If the Flight controller misses three uplinks, it will enter a landing mode

#define LORA_SETSLEEP 0x84
#define LORA_SETSTANDBY 0x80
#define LORA_SETFS 0xC1
#define LORA_SETTX 0x83
#define LORA_SETRX 0x82
#define LORA_SETREGULATOR 0x96
#define LORA_CALIBRATE_IMAGE 0x98
#define LORA_SETPA 0x95

#define LORA_WRITE_REGISTER 0x0D
#define LORA_WRITE_BUFFER 0x0E
#define LORA_READ_BUFFER 0x1E

#define LORA_SET_RF_FREQ 0x86
#define LORA_SET_PACKET 0x8A
#define LORA_SET_MOD_PARAMS 0x8B
#define LORA_SET_PACKET_PARAMS 0x8C
#define LORA_SET_TX_PARAMS 0x8E
#define LORA_SET_BUFFER_BASE_ADR 0x8F
#define LORA_SET_DIO_IRQ_PARAMS 0x08
#define LORA_CLEAR_IRQ_STATUS 0x02

#define LORA_GET_STATUS 0xC0
#define LORA_GET_IRQ_STATUS 0x12
#define LORA_GET_RX_BUFFER_STATUS 0x13
#define LORA_GET_PACKET_STATUS 0x14

#define LORA_RX_DONE_IRQ (1<<1)
#define LORA_TX_DONE_IRQ (1<<0)
// Includes $, *, and checksum
#define INTERCHANGE_DOWNLINK_SIZE 10
#define INTERCHANGE_DATA_SIZE 6
// Includes $, *, and checksum
#define UPLINK_SIZE 35
#define UPLINK_DATA_SIZE 31
#define RX_BASE_ADR 0
#define TX_BASE_ADR 100
#define LORA_SYNC_WORD 0x6494

#define LORA_STANDARD_DELAY 300000

typedef enum {
	// LORA is ready to transition modes
	LORA_Standby,
	// LORA is actively listening for uplinks
	LORA_Receiving,
	// LORA has built a downlink, ready to send
	LORA_Ready_to_Transmit,
	// LORA is ramping power and transmitting downlink
	LORA_Transmitting
} LORA_Status;

typedef struct {
	FC_Status Drone_status;
	float Desired_north;
	float Desired_east;
	float Desired_altitude;
	float Base_altitude;
} Uplink;

typedef struct {
	// Are we calibrated
	FC_Status Flight_Controller_Status;
	// Are we tracking the reference well
	uint8_t Tracking_Status;
	char ID[3];
} Downlink;

// Set radio frequency, antenna power, packet parameters, and interrupt flag bit mask
uint8_t Setup_LoRa();
// Receive and parse uplink
uint8_t Receive_Uplink(Uplink *inbound, Downlink *outbound);
// Respond to uplink with downlink
void Send_Downlink(Downlink *outbound);
// State machine to transition drone state
void Manage_FC_Status(FC_Status Desired);
// Get payload length in LoRa FIFO
uint8_t Check_For_Message(uint8_t *rx_offset);
// Delay and check LORA busy pin
void LORA_Delay(uint32_t length);
// Manage LORA state
void Run_LORA(Uplink *uplink);
// Given a string of characters, returns a compound xor checksum in character hex format
void Xor_Checksum(char *data, uint8_t length, uint8_t start_index, char checksum_hex[3]);

#endif

