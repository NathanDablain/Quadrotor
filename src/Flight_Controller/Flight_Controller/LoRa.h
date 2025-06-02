#ifndef LORA_H
#define LORA_H

// Protocol for 7th bit when using SPI interface:
// -> 1 is a write, 0 is a read

// Downlink message format -> $ND_MM_C_T*CS
// Downlink message size -> 10, '_' are for readability, not part of message
// Uplink message format -> $ND_MM_nnn.nn_N_eee.ee_E_hhh.hh_HHH.HH_C*CS
// Uplink message size -> 35, '_' are for readability, not part of message
// Downlink MM should match most recent Uplink MM

// Ground Controller sends an uplink at 1Hz, Flight controller should respond with a downlink upon receiving the uplink
// If the Flight controller misses three uplinks, it will enter a landing mode

#define LORA_REG_OP_MODE 0x01
#define LORA_REG_F_MSB 0x06
#define LORA_REG_F_MIDB 0x07
#define LORA_REG_F_LSB 0x08
#define LORA_REG_RX_N_BYTES 0x13
#define LORA_REG_FIFO_ADR_PTR 0x0D
#define LORA_REG_RX_ADR 0x10
#define LORA_REG_TX_ADR 0x0E
#define LORA_REG_FIFO 0x00
#define LORA_REG_PAYLOAD_LENGTH 0x22
#define LORA_REG_PA_CONFIG 0x09
#define LORA_REG_IRQ_FLAGS_MASK 0x11
#define LORA_REG_IRQ_FLAGS 0x12
#define LORA_REG_OCP 0x0B
#define LORA_REG_PA_RAMP 0x0A

#define LORA_MODE_SLEEP 0b10000000
#define LORA_MODE_STDBY 0b10000001
#define LORA_MODE_RXCONTINUOUS 0b10000101
#define LORA_MODE_TX 0b10000011

#define LORA_FREQ_915_HB 0b11100100
#define LORA_FREQ_915_MB 0b11000000
#define LORA_FREQ_915_LB 0b00000000

#define LORA_PA_20dBm 0b11111111
#define LORA_PA_14dBm 0b01111111

#define LORA_MASK_TX 0b11110111
#define LORA_IRQ_TX_DONE 0b00001000

#define DOWNLINK_SIZE 10
#define UPLINK_SIZE 35

typedef enum {
	// Drone systems initialized, awaiting calibration
	Standby,
	// Drone systems calibrating
	Calibrating,
	// Drone systems are calibrated, ready to fly
	Ready,
	// Drone is flying, responding to commands and under autopilot control
	Flying,
	// Drone is following landing procedure, will automatically proceed to ready once complete
	Landing
} FC_Status;

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
	unsigned char Tracking_Status;
	char ID[3];
} Downlink;

// Set radio frequency, antenna power, and interrupt flag bit mask
unsigned char Setup_LoRa();
// Receive and parse uplink
void Receive_Uplink(Uplink *inbound, Downlink *outbound, FC_Status *Flight_Controller_Status);
// Respond to uplink with downlink
void Send_Downlink(Downlink *outbound);
// State machine to transition drone state
FC_Status Manage_FC_Status(FC_Status Desired, FC_Status Current);
// Get payload length in LoRa FIFO
unsigned char Check_For_Message();

#endif