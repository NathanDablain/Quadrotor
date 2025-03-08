#ifndef LORA_H
#define LORA_H
// Once a second, sample lora fifo for messages. Messages should contain
// -> Base height to compare with bar readings
// -> Target NED position
// -> Status command - on,off,
// Messages will mimic NMEA format -> $ND,mmmm.mm,nnn.nn,N,eee.ee,E,hhh.hh,S*CS

#define LORA_REG_OP_MODE 0x01
#define LORA_REG_F_MSB 0x06
#define LORA_REG_F_MIDB 0x07
#define LORA_REG_F_LSB 0x08
#define LORA_REG_RX_N_BYTES 0x13
#define LORA_REG_FIFO_ADR_PTR 0x0D
#define LORA_REG_RX_ADR 0x10
#define LORA_REG_FIFO 0x00
#define LORA_REG_PAYLOAD_LENGTH 0x22
#define LORA_REG_PA_CONFIG 0x09

// 1 for write, 0 for read
unsigned char 
	Setup_LoRa();
unsigned int
	Read_LoRa(States *Reference);

#endif