#ifndef LORA_P32_H
#define LORA_P32_H

#include <stdint.h>
#include "Sim_Types.h"

// Includes $, *, and checksum
#define DOWNLINK_SIZE 10
#define DOWNLINK_DATA_SIZE 6
#define UPLINK_SIZE 35
#define UPLINK_DATA_SIZE 31

#define ALTITUDE_CEILING 5.0

typedef enum {
	// LORA is ready to transition modes
	LORA_Standby,
	// LORA is actively listening for uplinks
	LORA_Receiving,
	// LORA has built a downlink, ready to send
	LORA_Ready_to_Transmit,
	// LORA is ramping power and transmitting downlink
	LORA_Transmitting,
    // LORA did not initialize properly
    LORA_Fail
} LORA_Status;

typedef struct {
	FC_Status Drone_status;
	double Desired_north;
	double Desired_east;
	double Desired_altitude;
	double Base_altitude;
} Uplink;

typedef struct {
	// Are we calibrated
	FC_Status Flight_Controller_Status;
	// Are we tracking the reference well
	uint8_t Tracking_Status;
	char ID[3];
} Downlink;

void Initialize_LORA();

void Run_LORA();
// Receive and parse uplink
uint8_t Receive_Uplink();
// Respond to uplink with downlink
void Send_Downlink();
// State machine to transition drone state
void Manage_FC_Status(FC_Status Desired);
// Given a string of characters, returns a compound xor checksum in character hex format
void Xor_Checksum(char *data, uint8_t length, uint8_t start_index, char checksum_hex[3]);
// Returns the uplinked desired altitude
double Uplink_Altitude();

#endif