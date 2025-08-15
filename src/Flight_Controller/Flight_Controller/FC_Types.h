#ifndef FC_TYPES
#define FC_TYPES

// Change device to change compilation related to various peripherals - AVR128DB48 and AVR64DA28
#define AVR64DA28
//#define AVR128DB48

// ADC_MUX_ESC - Positive mux input to ADC, set to ESC voltage pin
// ADC_PIN_CTRL - ESC voltage pin interrupt control 
#if defined(AVR128DB48)
	#define ADC_MUX_ESC					ADC_MUXPOS_AIN6_gc
	#define ADC_PIN_CTRL				PORTD_PIN6CTRL
	#define ADC_PIN						PIN6_bm
	#define PORT_MAG					PORTB
	#define CS_MAG						PIN3_bm
	#define PORT_BAR					PORTA
	#define CS_BAR						PIN7_bm
	#define PORT_IMU					PORTA
	#define CS_IMU						PIN6_bm
	#define PORT_LORA					PORTA
	#define CS_LORA						PIN5_bm
	#define PRIMARY_SPI					SPI1
	#define PRIMARY_SPI_PORT			PORTC
	#define PRIMARY_SPI_MOSI_PIN		PIN0_bm
	#define PRIMARY_SPI_SCK_PIN			PIN2_bm
	#define PRIMARY_USART				USART3
	#define PRIMARY_USART_RXC_VECT		USART3_RXC_vect
	#define PRIMARY_USART_PORT			PORTB
	#define PRIMARY_USART_TX_PIN		PIN1_bm
	#define PRIMARY_TWI_PORT			PORTA
	#define MOTOR1_PIN					PIN0_bm
	#define MOTOR2_PIN					PIN1_bm
	#define MOTOR3_PIN					PIN2_bm
	#define MOTOR4_PIN					PIN3_bm	
#elif defined(AVR64DA28)
	#define ADC_MUX_ESC					ADC_MUXPOS_AIN2_gc
	#define ADC_PIN_CTRL				PORTD_PIN2CTRL
	#define ADC_PIN						PIN2_bm
	#define PORT_MAG					PORTA
	#define CS_MAG						PIN2_bm
	#define PORT_BAR					PORTA
	#define CS_BAR						PIN0_bm
	#define PORT_IMU					PORTA
	#define CS_IMU						PIN1_bm
	#define PORT_LORA					PORTA
	#define CS_LORA						PIN3_bm
	#define PRIMARY_SPI					SPI0
	#define PRIMARY_SPI_PORT			PORTA
	#define PRIMARY_SPI_MOSI_PIN		PIN4_bm
	#define PRIMARY_SPI_SCK_PIN			PIN6_bm
	#define PRIMARY_USART				USART2
	#define PRIMARY_USART_RXC_VECT		USART2_RXC_vect
	#define PRIMARY_USART_PORT			PORTF
	#define PRIMARY_USART_TX_PIN		PIN0_bm
	#define PRIMARY_TWI_PORT			PORTC
	#define MOTOR1_PIN					PIN3_bm
	#define MOTOR2_PIN					PIN4_bm
	#define MOTOR3_PIN					PIN5_bm
	#define MOTOR4_PIN					PIN6_bm
#endif

// Global variables
extern volatile unsigned long g_seconds;
// Tracks when to check LoRa for uplink, incremented at 200 Hz
extern volatile unsigned char g_LoRa_Check_Flag;
// Tracks when to sample magnetometer
extern volatile unsigned char g_MAG_Read_Flag;
// Tracks when to sample imu
extern volatile unsigned char g_Accel_Read_Flag;
// Tracks when to sample imu
extern volatile unsigned char g_Gyro_Read_Flag;
// Tracks when to sample barometer
extern volatile unsigned char g_BAR_Read_Flag;
// Tracks how often to send PPM signal to ESC, driving motor throttles
extern volatile unsigned char g_Motor_Run_Flag;
// Tracks how often to run altitude MRAC, drives desired thrust
extern volatile unsigned char g_Altitude_Control_Flag;
// Tracks how often to run guidance function
extern volatile unsigned char g_Guidance_Flag;
// Tracks when to use accelerometer and magnetometer data to include measurements
extern volatile unsigned char g_Attitude_Observer_Update_Flag;
// Tracks when to use gyro data to make predictions
extern volatile unsigned char g_Attitude_Observer_Predict_Flag;
// Tracks when power is first applied to the motors
extern volatile unsigned char g_Motor_Power_Flag;
// Motor_Throttles-> Values from 0-1000 with 1000 being max throttle, motor order is: back, left, right, front
extern volatile unsigned int g_Motor_Throttles[4];
// Indicates motors are calibrated
extern volatile unsigned char g_Motor_Cal_Flag;
// Watchdog times out after 3 seconds without receiving a readable up link, triggers safety switch
extern volatile unsigned char g_positive_coms_watchdog;

#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)

// Structures
typedef struct{
	signed int w[3];
	signed int g_vec[3]; // In the frame Forward - Right - Down
	float m_vec[3];
	signed long m_xyz_LSB[3];
	float Euler[3];
	float Pressure_Altitude;
	float Compensated_Height;
	float Velocity_NED[3];
	float Position_NED[3];
	signed long Longitude;
	signed long Latitude;
	float Position_ECEF[3];
	float Speed_over_ground;
	float Course_over_ground;
} States;

typedef struct{
	float Position_NED[3];
	float Euler[3];
} Reference;

typedef struct{
	// bar_cal_status -> flag with state of calibration, 0: uncalibrated, 1:ready
	// altitude_bias -> offset to apply against bar reading, either from ground controller or last good reading
	unsigned char bar_cal_status;
	float altitude_bias;
	// imu_cal_status -> flag with state of calibration, 0: uncalibrated, 1: ready
	// w_bias -> constant offset along each gyro axis in LSB
	unsigned char imu_cal_status;
	signed int w_bias[3];
	// Magnetometer cal data
	// mag_cal_status -> flag with state of calibration, 0: uncalibrated, 1: partial calibration, 2: ready
	// m_max -> max magnetic field recorded along each axis
	// m_min -> min magnetic field recorded along each axis
	// hard_iron -> offset incurred by nearby hard iron sources, shifts local field off 0 mean
	unsigned char mag_cal_status;
	signed int m_max[3];
	signed int m_min[3];
	signed int hard_iron[3];
	// GPS cal data
	// gps_cal_status -> flag with state of calibration, 0: uncalibrated, 1: ready
	// Reference_Position_ecef -> Earth Centered Earth Fixed coordinates of initial reference position which NED offset is based on
	unsigned char gps_cal_status;
	float Reference_Position_ecef[3];
	// Motor cal data
	unsigned char motor_cal_status;
	unsigned char motor_cal_flags[4];
} Calibration_Data;

void Delay(unsigned long long length);


#endif