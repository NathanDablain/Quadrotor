#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "time.h"
#include "spi.h"
#include "dma.h"
#include "imu.h"
#include "global_variables.h"
#include "pins.h"

static IMU_Data imu;
static IMU_Machine state_accel;
static IMU_Machine state_gyro;

void Initialize_IMU_Machine(){
    memset(&imu, 0, sizeof(imu));
    imu.drdy_Flag_Accel = false;
    imu.drdy_Flag_Gyro = false;
    imu.Accel_Read_Array[0] = ACCEL_DATA_START|0x80;
    imu.Gyro_Read_Array[0] = GYRO_DATA_START|0x80;
    state_accel = Initialize_IMU();
    state_gyro = state_accel;
}

void Run_IMU_Machine(){
    const int32_t ODR_Accel_Hz = 1666;
    const int32_t ODR_Gyro_Hz = 3332;
    const Time Sample_Rate_Accel = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Accel_Hz};
    const Time Sample_Rate_Gyro = {.seconds = 0, .tmr1_count = g_tmr1_ct_in_s/ODR_Gyro_Hz};

    // Accelerometer state machine
    switch (state_accel){
       case IMU_Standby:
           state_accel = Initialize_IMU();
           break;
       case IMU_Fail:
           break;
       case IMU_Ready:
           if ((g_spi1_rdy_flag) && (Compare_And_Update(Current_Time(), Sample_Rate_Accel, &imu.Last_Update_Accel))){
               Prepare_SPI1_For_DMA(&CS_IMU_PORT, CS_IMU_PIN, imu.Accel_Read_Array, &imu.drdy_Flag_Accel);
               Set_DMA_01(&imu.Accel_Read_Array[1], &imu.accel_LSB_bytes[0], sizeof(imu.Accel_Read_Array));
               state_accel = IMU_Reading;
           }
           break;
       case IMU_Reading:
           if (imu.drdy_Flag_Accel){
               Convert_Accel();
               imu.drdy_Flag_Accel = false;
               state_accel = IMU_Ready;
           }
           break;
   }
     
    // Gyroscope state machine
    switch (state_gyro){
       case IMU_Standby:
           state_gyro = state_accel;
           break;
       case IMU_Fail:
           break;
       case IMU_Ready:
           if ((g_spi1_rdy_flag) && (Compare_And_Update(Current_Time(), Sample_Rate_Gyro, &imu.Last_Update_Gyro))){
               Prepare_SPI1_For_DMA(&CS_IMU_PORT, CS_IMU_PIN, imu.Gyro_Read_Array, &imu.drdy_Flag_Gyro);
               Set_DMA_01(&imu.Gyro_Read_Array[1], &imu.gyro_LSB_bytes[0], sizeof(imu.Gyro_Read_Array));
               state_gyro = IMU_Reading;
           }
           break;
       case IMU_Reading:
           if (imu.drdy_Flag_Gyro){
               Convert_Gyro();
               imu.drdy_Flag_Gyro = false;
               state_gyro = IMU_Ready;
           }
           break;
   }
}

IMU_Machine Initialize_IMU(){
   // 0 for write 1 for read
    uint8_t data_in[2] = {IMU_WHO_AM_I | 0x80, 0};
    uint8_t data_out[2] = {0};
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
	if (data_out[1] != IMU_ID) return IMU_Fail;

    // Set accelerometer range to +-2g, ODR to 1660 Hz, enable high resolution
    data_in[0] = IMU_CTRL1_XL;
    data_in[1] = IMU_2G_RANGE | IMU_ODR_1660HZ | IMU_ACCEL_HIGH_RES;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));

    // Set gyro range to +-500 dps, ODR to 3330 Hz
    data_in[0] = IMU_CTRL2_G;
    data_in[1] = IMU_500_RANGE | IMU_ODR_3330HZ;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
    
    // Enable Gyro LPF
    data_in[0] = IMU_CTRL4_C;
    data_in[1] = IMU_GYRO_LPF_EN;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
    
    // Set gyroscope LPF BW to 153 Hz and enable high performance mode for accel
    data_in[0] = IMU_CTRL6_C;
    data_in[1] = IMU_ACCEL_HP_MODE | IMU_GYRO_BW2;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
    
    // Set accelerometer LPF to ODR/400
    data_in[0] = IMU_CTRL8_XL;
    data_in[1] = IMU_ACCEL_BW_400;
    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
    
    
//    uint8_t data_in[2] = {IMU_CTRL3_C, IMU_RESET};
//    uint8_t data_out[2] = {0};
//    
//    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
//    // Give device time to reboot before setting parameters
//	Delay(10000);
//    
//    data_in[0] = (IMU_WHO_AM_I | 0x80);
//    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
//	if (data_out[1] != IMU_ID) return IMU_Fail;
//    
//    // Set accelerometer range to +-2g, ODR to 1660 Hz, enable low pass filter
//    data_in[0] = IMU_CTRL1_XL;
//    data_in[1] = IMU_2G_RANGE | IMU_ODR_1660HZ | IMU_ACCEL_LPF_EN;
//    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
//
//    // Set gyro range to +-500 dps, ODR to 1660 Hz
//    data_in[0] = IMU_CTRL2_G;
//    data_in[1] = IMU_500_RANGE | IMU_ODR_1660HZ;
//    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
//
//    // Enable gyro low pass filter
//    data_in[0] = IMU_CTRL4_C;
//    data_in[1] = IMU_GYR_LPF_EN;
//    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
//
//    // Set gyro low pass filter bandwidth to 168Hz
//    data_in[0] = IMU_CTRL6_C;
//    data_in[1] = IMU_GYR_LPF_BW2;
//    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
//    
//    // Set gyro high pass filter and normal mode
////    data_in[0] = IMU_CTRL7_G;
////    data_in[1] = IMU_G_HPM | IMU_G_HPF;
////    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
//
//    // Set accelerometer low pass filter bandwidth to ODR/9
//    data_in[0] = IMU_CTRL8_XL;
//    data_in[1] = IMU_ACC_LPF_BW3;
//    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));
//
//     // Set block data update and auto increment
//    data_in[0] = IMU_CTRL3_C;
//    data_in[1] = IMU_BDU | IMU_AUTO_INC;
//    SPI_transfer(&CS_IMU_PORT, CS_IMU_PIN, data_in, data_out, sizeof(data_in));

    return IMU_Ready;
}

void Convert_Accel(){
    // The IMU sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = -Sensor z
	static int16_t accel_max[3] = {0};
    static int16_t accel_min[3] = {0};
    
    imu.accel_LSB[0] = (((int16_t)imu.accel_LSB_bytes[2])<<8) + ((int16_t)imu.accel_LSB_bytes[1]);
    imu.accel_LSB[1] = -(((int16_t)imu.accel_LSB_bytes[4])<<8) - ((int16_t)imu.accel_LSB_bytes[3]);
    imu.accel_LSB[2] = -(((int16_t)imu.accel_LSB_bytes[6])<<8) - ((int16_t)imu.accel_LSB_bytes[5]);

    imu.acceleration[0] = ((double)imu.accel_LSB[0])*ACCEL_SENS;
    imu.acceleration[1] = ((double)imu.accel_LSB[1])*ACCEL_SENS;
    imu.acceleration[2] = ((double)imu.accel_LSB[2])*ACCEL_SENS;
    
    for (uint8_t i = 0; i < 3; i++){
		bool calculate_offset = false;
		if (imu.accel_LSB[i] > accel_max[i]){
			accel_max[i] = imu.accel_LSB[i];
			calculate_offset = true;
		}
		else if (imu.accel_LSB[i] < accel_min[i]){
			accel_min[i] = imu.accel_LSB[i];
			calculate_offset = true;
		}
		if (calculate_offset){
            imu.accel_bias_LSB[i] = (int32_t)accel_max[i] + (int32_t)accel_min[i];
			imu.accel_bias_LSB[i] >>= 1;
		}
	}
}

void Convert_Gyro(){
    // The IMU sensor axis corresponds to the body axis in the following way
    // -> Body x = Sensor x
    // -> Body y = -Sensor y
    // -> Body z = -Sensor z

    imu.gyro_LSB[0] = (((int16_t)imu.gyro_LSB_bytes[2])<<8) + ((int16_t)imu.gyro_LSB_bytes[1]);
    imu.gyro_LSB[1] = -(((int16_t)imu.gyro_LSB_bytes[4])<<8) - ((int16_t)imu.gyro_LSB_bytes[3]);
    imu.gyro_LSB[2] = -(((int16_t)imu.gyro_LSB_bytes[6])<<8) - ((int16_t)imu.gyro_LSB_bytes[5]);
    
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