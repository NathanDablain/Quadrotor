#include <stdbool.h>
#include <stdint.h>
#include "time.h"
#include "spi.h"
#include "dma.h"
#include "imu.h"
#include "global_variables.h"
#include "pins.h"

static IMU_Data imu = {0};
static IMU_Machine state_accel = IMU_Standby;
static IMU_Machine state_gyro = IMU_Standby;

void Run_IMU_Machine(){
    const int32_t ODR_Accel_Hz = 1660;
    const int32_t ODR_Gyro_Hz = 1660;
    const Time Sample_Rate_Accel = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Accel_Hz};
    const Time Sample_Rate_Gyro = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Gyro_Hz};
    static uint8_t Accel_Read_Array[7] = {0};
    static uint8_t Gyro_Read_Array[9] = {0};
    static Time Last_Update_Accel = {0};
    static Time Last_Update_Gyro = {0};
    static bool Data_Ready_Flag_Accel = false;
    static bool Data_Ready_Flag_Gyro = false;

    // Accelerometer state machine
    switch (state_accel){
       case IMU_Standby:
           Accel_Read_Array[0] = ACCEL_DATA_START|0x80;
           state_accel = Initialize_IMU();
           break;
       case IMU_Fail:
           break;
       case IMU_Ready:
           if ((g_spi1_rdy_flag) && (Compare_And_Update(Current_Time(), Sample_Rate_Accel, &Last_Update_Accel))){
               Prepare_SPI1_For_DMA(&CS_IMU_PORT, CS_IMU_PIN, Accel_Read_Array, &Data_Ready_Flag_Accel);
               Set_DMA_01(&Accel_Read_Array[1], &imu.accel_LSB_bytes[0], sizeof(Accel_Read_Array));
               state_accel = IMU_Reading;
           }
           break;
       case IMU_Reading:
           if (Data_Ready_Flag_Accel){
               Convert_Accel();
               Data_Ready_Flag_Accel = false;
               state_accel = IMU_Ready;
           }
           break;
   }
     
    // Gyroscope state machine
    switch (state_gyro){
       case IMU_Standby:
           Gyro_Read_Array[0] = GYRO_DATA_START|0x80;
           state_gyro = state_accel;
           break;
       case IMU_Fail:
           break;
       case IMU_Ready:
           if ((g_spi1_rdy_flag) && (Compare_And_Update(Current_Time(), Sample_Rate_Gyro, &Last_Update_Gyro))){
               Prepare_SPI1_For_DMA(&CS_IMU_PORT, CS_IMU_PIN, Gyro_Read_Array, &Data_Ready_Flag_Gyro);
               Set_DMA_01(&Gyro_Read_Array[1], &imu.gyro_LSB_bytes[0], sizeof(Gyro_Read_Array));
               state_gyro = IMU_Reading;
           }
           break;
       case IMU_Reading:
           if (Data_Ready_Flag_Gyro){
               Convert_Gyro();
               Data_Ready_Flag_Gyro = false;
               state_gyro = IMU_Ready;
           }
           break;
   }
}

IMU_Machine Initialize_IMU(){
    uint8_t data_in[2] = {IMU_CTRL3_C, IMU_RESET};
    uint8_t data_out[2] = {0};
    
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
    // Give device time to reboot before setting parameters
	Delay(10000);
    
    data_in[0] = (IMU_WHO_AM_I | 0x80);
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
	if (data_out[1] != IMU_ID) return IMU_Fail;
    
    // Set accelerometer range to +-2g, ODR to 1660 Hz, enable low pass filter
    data_in[0] = IMU_CTRL1_XL;
    data_in[1] = IMU_2G_RANGE | IMU_ODR_1660HZ | IMU_ACCEL_LPF_EN;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));

    // Set gyro range to +-500 dps, ODR to 1660 Hz
    data_in[0] = IMU_CTRL2_G;
    data_in[1] = IMU_500_RANGE | IMU_ODR_1660HZ;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));

    // Enable gyro low pass filter
    data_in[0] = IMU_CTRL4_C;
    data_in[1] = IMU_GYR_LPF_EN;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));

    // Set gyro low pass filter bandwidth to 505Hz
    data_in[0] = IMU_CTRL6_C;
    data_in[1] = IMU_GYR_LPF_BW3;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
    
    // Set gyro high pass filter and normal mode
    data_in[0] = IMU_CTRL7_G;
    data_in[1] = IMU_G_HPM | IMU_G_HPF;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));

    // Set accelerometer low pass filter bandwidth to ODR/9
    data_in[0] = IMU_CTRL8_XL;
    data_in[1] = IMU_ACC_LPF_BW2;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));

     // Set block data update and auto increment
    data_in[0] = IMU_CTRL3_C;
    data_in[1] = IMU_BDU | IMU_AUTO_INC;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));

    return IMU_Ready;
}

void Convert_Accel(){
    // The IMU sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = -Sensor z

    imu.accel_LSB[0] = (((int16_t)imu.accel_LSB_bytes[2])<<8) + ((int16_t)imu.accel_LSB_bytes[1]);
    imu.accel_LSB[1] = -(((int16_t)imu.accel_LSB_bytes[4])<<8) - ((int16_t)imu.accel_LSB_bytes[3]);
    imu.accel_LSB[2] = -(((int16_t)imu.accel_LSB_bytes[6])<<8) - ((int16_t)imu.accel_LSB_bytes[5]);

    imu.acceleration[0] = ((double)imu.accel_LSB[0])*ACCEL_SENS;
    imu.acceleration[1] = ((double)imu.accel_LSB[1])*ACCEL_SENS;
    imu.acceleration[2] = ((double)imu.accel_LSB[2])*ACCEL_SENS;
}

void Convert_Gyro(){
    // The IMU sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = -Sensor z

    imu.gyro_LSB[0] = (((int16_t)imu.gyro_LSB_bytes[4])<<8) + ((int16_t)imu.gyro_LSB_bytes[3]);
    imu.gyro_LSB[1] = -(((int16_t)imu.gyro_LSB_bytes[6])<<8) - ((int16_t)imu.gyro_LSB_bytes[5]);
    imu.gyro_LSB[2] = -(((int16_t)imu.gyro_LSB_bytes[8])<<8) - ((int16_t)imu.gyro_LSB_bytes[7]);
    
    imu.angular_rate[0] = ((double)imu.gyro_LSB[0])*GYRO_SENS;
    imu.angular_rate[1] = ((double)imu.gyro_LSB[1])*GYRO_SENS;
    imu.angular_rate[2] = ((double)imu.gyro_LSB[2])*GYRO_SENS;
}

double IMU_Acceleration(uint8_t index){
    return imu.acceleration[index];
}

double IMU_Angular_Rate(uint8_t index){
    return imu.angular_rate[index];
}