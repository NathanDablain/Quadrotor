#ifndef BAROMETER_H
#define	BAROMETER_H

#include <stdint.h>

typedef enum{
    BAR_Standby,
    BAR_Fail,
    BAR_Ready,
    BAR_Reading
} BAR_Machine;

typedef struct{
    uint8_t pressure_LSB_bytes[4];
    uint32_t pressure_LSB;
    uint8_t pressure_offset;
    double pressure_pa;
    double height;
    double base_altitude;
    uint8_t BAR_Read_array[4];
    Time Last_Update;
    bool drdy_flag;
} BAR_Data;

#define BAR_WHO_AM_I 0x0F
#define BAR_ID 0b10110011

#define BAR_IF_CTRL 0x0E

#define BAR_CTRL_REG1 0x10
#define BAR_ODR_200 (0b111 << 4)
#define BAR_ODR_100 (0b110 << 4)
#define BAR_ODR_75 (0b101 << 4)
#define BAR_ODR_50 (0b100 << 4)
#define BAR_LPF (1 << 3)
#define BAR_LPF_CFG (1 << 2)
#define BAR_BDU (1 << 1)

#define BAR_CTRL_REG2 0x11
#define BAR_BOOT (1 << 7)
#define BAR_ADD_INC (1 << 4)
#define BAR_RST (1 << 2)
#define BAR_LOW_NOISE (1 << 1)

#define BAR_DATA_START 0x28
#define BAR_FIFO_DATA_START 0x78
#define BAR_FIFO_WTM 0x14
#define BAR_STATUS 0x27
#define BAR_FIFO_CTRL 0x13
#define BAR_FIFO_STATUS1 0x25
#define BAR_P_DRDY_bm 0x01

void Initialize_Barometer_Machine();

void Run_Barometer_Machine();

BAR_Machine Initialize_Barometer();

void Convert_Pressure();

double Barometer_Altitude();

double Barometer_Pressure();

uint32_t Barometer_Pressure_LSB();

#endif

