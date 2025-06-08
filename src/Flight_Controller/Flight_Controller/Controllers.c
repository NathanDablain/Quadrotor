#include "Controllers.h"

volatile unsigned char g_Motor_Power_Flag = 0;
volatile Motors g_Motor_Pins = {.Pins = {PIN0_bm, PIN1_bm, PIN2_bm, PIN3_bm}, .index = 0};

void Calibrate_Motors(Calibration_Data *cal_data, unsigned int Motor_Throttles[4]){
	//unsigned long counter = 1000;
	g_Motor_Power_Flag = 0;
	//while (--counter){
		//for (unsigned char i = 0; i < 4; i++){
			//Motor_Throttles[i] = counter;
		//}
		//Delay(2000);
	//}
	memset(Motor_Throttles, 0, 8);
	cal_data->motor_cal_status = 1;
}

void Safety_Check(States *Drone, unsigned int motor_throttles[4], FC_Status *Flight_Controller_Status){
	unsigned char safety_switch = 0;
	
	if (fabs(Drone->Euler[0]) > MOTOR_CUTOFF_ANGLE) safety_switch = 1;
	if (fabs(Drone->Euler[1]) > MOTOR_CUTOFF_ANGLE) safety_switch = 1;
	if (fabs(Drone->Position_NED[2]) > MOTOR_CUTOFF_ALTITUDE) safety_switch = 1;
	
	if (safety_switch){
		memset(motor_throttles, 0, 8);
		*Flight_Controller_Status = Standby;
	}
}
	
volatile unsigned char g_Guidance_Flag = 0;

void Run_Guidance(Reference *Desired_States, Reference *Commanded_States){
	g_Guidance_Flag = 0;
	// IIR to prevent large jumps in reference states
	const float c1 = 0.99;
	const float c2 = 1.0 - c1;
	for (unsigned char i = 0; i < 3; i++){
		Commanded_States->Euler[i] = Commanded_States->Euler[i]*c1 + Desired_States->Euler[i]*c2;
		Commanded_States->Position_NED[i] = Commanded_States->Position_NED[i]*c1 + Desired_States->Position_NED[i]*c2;
	}
}

volatile unsigned char g_Altitude_Control_Flag = 0;

float Altitude_Control(float h, float h_ref){
	g_Altitude_Control_Flag = 0;
	// Gains K chosen through pole placement of double integrator
	const float mass = 0.45;
	const float d_t = 0.01;
	const float IIR_c1 = 0.9;
	const float IIR_c2 = 1.0 - IIR_c1;
	const float K[2] = {3.0, 5.0};
	const float K_int = 0.002;
	static float e_int;
	static float h_last;
	static float h_dot_last;
	e_int += (h_ref-h);
	float h_dot = h_dot_last*IIR_c1 + ((h-h_last)/d_t)*IIR_c2;
	h_dot_last = h_dot;
	h_last = h;
	float u = -K[0]*(h-h_ref) - K[1]*h_dot + K_int*e_int + 9.81;
	float thrust = mass*u;
    return thrust;
}

void Euler_Control(float Current_Euler[3], float Commanded_Euler[3], float desired_moments[3]){
    const float d_t = 0.0025;
	const float I[3] = {0.00149, 0.00262, 0.00149};
    const float K[3][2] = {{100, 200},{100, 200},{10, 20}};
	const float K_int = 0.1;
	const float IIR_c1 = 0.9;
	const float IIR_c2 = 1.0 - IIR_c1;
    static float Euler_last[3];
	static float Euler_dot_last[3];
	static float e_int[3];
    for (unsigned char i = 0; i < 3; i++){
	    float e = Commanded_Euler[i] - Current_Euler[i];
		e_int[i] += e;
	    float Euler_dot = Euler_dot_last[i]*IIR_c1 + ((Current_Euler[i]-Euler_last[i])/d_t)*IIR_c2;
	    Euler_last[i] = Current_Euler[i];
		Euler_dot_last[i] = Euler_dot;
		float u = K[i][0]*e - K[i][1]*Euler_dot + K_int*e_int[i];
	    desired_moments[i] = u*I[i];
    }
}

void Set_throttles(unsigned int motor_throttles[4], float desired_thrust, float desired_moments[3]){
// Motor mixer
// Inputs - Desired thrust and body torques
// Outputs - Desired throttle command on each of 4 BLDC motors
    // -> Back motor (0) produces negative pitching torque and negative yawing torque
    // -> Left motor (1) produces positive rolling torque and positive yawing torque
    // -> Right motor (2) produces negative rolling torque and positive yawing torque
    // -> Front motor (3) produces positive pitching torque and negative yawing torque
	// Motor torque constant in N-m/A (1/KV)
	const float KT = 0.006366198;
	// This array holds the current produced by the motor for each 10% of throttle, starting at 0%
	const float Current[11] = {0.0, 0.1, 0.7, 2.0, 4.1, 7.2, 10.9, 15.4, 20.5, 25.9, 31.8};
    // Propeller thrust constant in N/(rad/s)^2 
    const float k_f = 0.000001;
    // Propeller torque constant in N-m/(rad/s)^2
    const float k_t = 0.000000011;
	// Gain to convert (rad/s)^2 to A
	const float K = KT/k_t;
    // Distance from front and back motor thrust vectors to drone center of gravity in (m)
    const float length_f_b =  0.117;
    // Distance from left and right motor thrust vectors to drone center of gravity in (m)
	const float length_l_r = 0.1205;
	const float denom_1 = 4.0*k_f*k_t*length_f_b;
	const float denom_2 = 4.0*k_f*k_t*length_l_r;
	const float c1 = k_f*length_f_b;
	const float c2 = k_t*length_f_b;
	const float c3 = 2.0*k_t;
	const float c4 = k_f*length_l_r;
	const float c5 = k_t*length_l_r;

	// w_f: (2*My*kt - Mz*kf*lfb + T*kt*lfb)/(4*kf*kt*lfb)
	// w_r: (Mz*kf*lrl - 2*Mx*kt + T*kt*lrl)/(4*kf*kt*lrl)
	// w_l: (2*Mx*kt + Mz*kf*lrl + T*kt*lrl)/(4*kf*kt*lrl)
	// w_b: -(2*My*kt + Mz*kf*lfb - T*kt*lfb)/(4*kf*kt*lfb)

	float omega_front = ((c3*desired_moments[1]) - (c1*desired_moments[2]) + (c2*desired_thrust))/denom_1;
	float omega_right = ((c4*desired_moments[2]) - (c3*desired_moments[0]) + (c5*desired_thrust))/denom_2;
	float omega_left = ((c3*desired_moments[0]) + (c4*desired_moments[2]) + (c5*desired_thrust))/denom_2;
	float omega_back = -((c3*desired_moments[1]) + (c1*desired_moments[2]) - (c2*desired_thrust))/denom_1;

	float omega[4] = {omega_back, omega_left, omega_right, omega_front};

	// Convert motor speeds in rad/s to throttle commands between 0-1000
	// omega = KT*i/k_t -> build mapping of i to throttle
	unsigned char first;
	unsigned char last;
	unsigned char middle;
	float I;
	// Perform binary search to interpolate current
	for (unsigned char i = 0; i < 4; i++){
		first = 0;
		last = 10;
		I = omega[i]/K;
		if (I < 0.0){
			motor_throttles[i] = 0;
			continue;
		}
		if (I > Current[10]){
			motor_throttles[i] = 1000;
			continue;
		}
		while(1){
			middle = (last - first)/2 + first;
			if (middle == first){
				break;
			}
			if (I < Current[middle]){
				last = middle;
				continue;
			}
			if (I > Current[middle]){
				first = middle;
			}
		}
		float temp = ((I - Current[first])/(Current[last]-Current[first]))*100;
		//float motor_throttle_iir = (float)motor_throttles[i]*0.9 +  (first*100 + temp)*0.1;
		motor_throttles[i] = (unsigned int)first*100 + (unsigned int)temp;
	}
}

volatile unsigned char g_Motor_Run_Flag = 0;

void Run_Motors(unsigned int Throttle_Commands[4]){
// ESC Interface - PPM (OneShot) control
// Inputs - Desired motor throttles (0-100)
// Outputs - 3500 Hz, 125-250 us waveform to ESC
	// We want to map 0:1000 to 1500:3000 (125:250 us)
	static unsigned int motor_lookup[1001] = {0};
	// Build the lookup table if it hasn't been built yet, enable pins for output
	if (!(motor_lookup[0])){ 
		for (unsigned int i=0;i<1001;i++){
			motor_lookup[i] = ((3*i)/2) + 1500;
		}
		PORTD_DIR |= PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm; 
	}
	unsigned int mapped_throttle_commands[4] = {0};
	// Map commands, saturate if out of bounds
	for (unsigned char i=0;i<4;i++){
		Throttle_Commands[i] = (Throttle_Commands[i]>1000)?1000:Throttle_Commands[i];
		mapped_throttle_commands[i] = motor_lookup[Throttle_Commands[i]];
	}
	// Disable Timer
	TCA0_SINGLE_CTRLA &= ~TCA_SINGLE_ENABLE_bm;
	// Set motor throttles
	TCA0_SINGLE_CMP0 = mapped_throttle_commands[0]; // Motor 1, back
	TCA0_SINGLE_CMP1 = mapped_throttle_commands[1]; // Motor 2, left
	TCA0_SINGLE_CMP2 = mapped_throttle_commands[2]; // Motor 3, right
	TCA1_SINGLE_CMP0 = mapped_throttle_commands[3]; // Motor 4, front
	// Reset timer counts
	TCA0_SINGLE_CNT = 0;
	TCA1_SINGLE_CNT = 0;
	// Set pins high
	PORTD_OUT |= PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
	// Start Timers
	TCA0_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;
	TCA1_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;
}

ISR(TCA0_CMP0_vect){
	// Set pin low
	PORTD_OUT &= ~PIN0_bm;
	// Clear int flag
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm;
}

ISR(TCA0_CMP1_vect){
	// Set pin low
	PORTD_OUT &= ~PIN1_bm;
	// Clear int flag
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP1_bm;
}

ISR(TCA0_CMP2_vect){
	// Set pin low
	PORTD_OUT &= ~PIN2_bm;
	// Clear int flag
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP2_bm;
}

ISR(TCA1_CMP0_vect){
	// Set pin low
	PORTD_OUT &= ~PIN3_bm;
	// Clear int flag
	TCA1_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm;
	// Only motor 4 uses TCA1, so disable timer
	TCA1_SINGLE_CTRLA &= ~TCA_SINGLE_ENABLE_bm;
}