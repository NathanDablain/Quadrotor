#include "Controllers.h"

volatile unsigned char g_Motor_Power_Flag = 0;
volatile unsigned char g_Guidance_Flag = 0;
volatile unsigned char g_Altitude_Control_Flag = 0;
volatile unsigned char g_Motor_Run_Flag = 0;
volatile unsigned char g_Motor_Cal_Flag = 0;
volatile unsigned int g_Motor_Throttles[4] = {0};
#if defined(AVR128DB48)
static unsigned char g_Motor1_Pin = PIN0_bm;
static unsigned char g_Motor2_Pin = PIN1_bm;
static unsigned char g_Motor3_Pin = PIN2_bm;
static unsigned char g_Motor4_Pin = PIN3_bm;
#elif defined(AVR64DA28)
static unsigned char g_Motor1_Pin = PIN6_bm;
static unsigned char g_Motor2_Pin = PIN5_bm;
static unsigned char g_Motor3_Pin = PIN4_bm;
static unsigned char g_Motor4_Pin = PIN3_bm;
#endif

Drone_Constants Initialize_Drone_Constants(){
	Drone_Constants Constants = {
	.k_f = 0.000001,
	.k_t = 0.000000011,
	.length_f_b =  0.117, // 0.137,
	.length_l_r = 0.1205, // 0.133, 
	.I = {0.00149, 0.00262, 0.00149}, // {0.00172, 0.00197, 0.00351},
	.mass = 0.45, // 0.50,
	.KT = 0.006366198,
	.K = 0.006366198/0.000000011,
	.Current = {0.0, 0.1, 0.7, 2.0, 4.1, 7.2, 10.9, 15.4, 20.5, 25.9, 31.8}};
		
	return Constants;
}

void Calibrate_Motors(Calibration_Data *cal_data){
	unsigned long counter = 1000;
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		g_Motor_Throttles[0] = 1000;
		g_Motor_Throttles[1] = 1000;
		g_Motor_Throttles[2] = 1000;
		g_Motor_Throttles[3] = 1000;
	}

	Delay(4000);
	while (--counter){
		for (unsigned char i = 0; i < 4; i++){
			ATOMIC_BLOCK(ATOMIC_FORCEON){
				 g_Motor_Throttles[i] = counter;
			}
		}
		while (!g_Motor_Cal_Flag);
		g_Motor_Cal_Flag = 0;
		//Delay(3000);
	}
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		 memset((unsigned int*)g_Motor_Throttles, 0, 8);
	}
	cal_data->motor_cal_status = 1;
	Delay(400000);
}

void Safety_Check(States *Drone, FC_Status *Flight_Controller_Status){
	unsigned char safety_switch = 0;
	
	if (fabs(Drone->Euler[0]) > MOTOR_CUTOFF_ANGLE) safety_switch = 1;
	if (fabs(Drone->Euler[1]) > MOTOR_CUTOFF_ANGLE) safety_switch = 1;
	if (fabs(Drone->Position_NED[2]) > MOTOR_CUTOFF_ALTITUDE) safety_switch = 1;
	
	if (safety_switch){
		ATOMIC_BLOCK(ATOMIC_FORCEON){
			memset((unsigned int*)g_Motor_Throttles, 0, 8);
		}
		*Flight_Controller_Status = Standby;
	}
}
	
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

float Altitude_Control(float h, float h_ref, const Drone_Constants *Constants){
	g_Altitude_Control_Flag = 0;
	// Gains K chosen through pole placement of double integrator
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
	float thrust = Constants->mass * u;
	
    return thrust;
}

void Euler_Control(float Current_Euler[3], float Commanded_Euler[3], float desired_moments[3], float thrust, const Drone_Constants *Constants){
    const float d_t = 0.0025;
    const float K[3][2] = {{100.0, 200.0},{100.0, 200.0},{10.0, 20.0}};
	const float K_int = 0.1;
	const float IIR_c1 = 0.9;
	const float IIR_c2 = 1.0 - IIR_c1;
	const float moment_split = 0.75;

	// Because motors cannot run backwards, there is a saturation point for how much moment can be applied that is a function of the thrust
	// Compute the omega (speed^2) expected of each motor to reach this thrust, this becomes maximum control authority for euler control

    static float Euler_last[3];
	static float Euler_dot_last[3];
	static float e_int[3];
	static unsigned char Saturation_Flag[3];
	
	float max_moment_phi = (thrust * Constants->length_l_r)/2.0;
	max_moment_phi *= moment_split;
	float max_moment_theta = (thrust * Constants->length_f_b)/2.0;
	max_moment_theta *= moment_split;
	float max_moment_psi = thrust*(Constants->k_t / Constants->k_f);
	max_moment_psi *= (1.0 - moment_split);
	
    for (unsigned char i = 0; i < 3; i++){
	    float e = Commanded_Euler[i] - Current_Euler[i];
		if ((Saturation_Flag[i] == 0) || ((Saturation_Flag[i] == 1) && (e < 0)) || ((Saturation_Flag[i] == 2) && (e > 0))){
			e_int[i] += e;
		}
	    float Euler_dot = Euler_dot_last[i]*IIR_c1 + ((Current_Euler[i]-Euler_last[i])/d_t)*IIR_c2;
	    Euler_last[i] = Current_Euler[i];
		Euler_dot_last[i] = Euler_dot;
		float u = K[i][0]*e - K[i][1]*Euler_dot + K_int*e_int[i];
	    desired_moments[i] = u * Constants->I[i];
    }
	
	Saturate(&desired_moments[0], max_moment_phi, -max_moment_phi, &Saturation_Flag[0]);
	Saturate(&desired_moments[1], max_moment_theta, -max_moment_theta, &Saturation_Flag[1]);
	Saturate(&desired_moments[2], max_moment_psi, -max_moment_psi, &Saturation_Flag[2]);
}

void Saturate(float *desired_moment, float max, float min, unsigned char* Saturation_Flag){
	if (*desired_moment > max){
		*desired_moment = max;
		*Saturation_Flag = 1;
	}
	else if (*desired_moment < min){
		*desired_moment = min;
		*Saturation_Flag = 2;
	}
	else{
		*Saturation_Flag = 0;
	}
}

void Set_throttles(float desired_thrust, float desired_moments[3], const Drone_Constants *Constants){
// Motor mixer
// Inputs - Desired thrust and body torques
// Outputs - Desired throttle command on each of 4 BLDC motors
    // -> Back motor (0) produces negative pitching torque and negative yawing torque
    // -> Left motor (1) produces positive rolling torque and positive yawing torque
    // -> Right motor (2) produces negative rolling torque and positive yawing torque
    // -> Front motor (3) produces positive pitching torque and negative yawing torque

	const float denom_1 = 4.0 * Constants->k_f * Constants->k_t * Constants->length_f_b;
	const float denom_2 = 4.0 * Constants->k_f * Constants->k_t * Constants->length_l_r;
	const float c1 = Constants->k_f * Constants->length_f_b;
	const float c2 = Constants->k_t * Constants->length_f_b;
	const float c3 = 2.0 * Constants->k_t;
	const float c4 = Constants->k_f * Constants->length_l_r;
	const float c5 = Constants->k_t * Constants->length_l_r;

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
		I = omega[i]/Constants->K;
		if (I < 0.0){
			ATOMIC_BLOCK(ATOMIC_FORCEON){
				g_Motor_Throttles[i] = 0;
			}
			continue;
		}
		if (I > Constants->Current[10]){
			ATOMIC_BLOCK(ATOMIC_FORCEON){
				g_Motor_Throttles[i] = 1000;
			}
			continue;
		}
		while(1){
			middle = (last - first)/2 + first;
			if (middle == first){
				break;
			}
			if (I < Constants->Current[middle]){
				last = middle;
				continue;
			}
			if (I > Constants->Current[middle]){
				first = middle;
			}
		}
		float temp = ((I - Constants->Current[first]) / (Constants->Current[last] - Constants->Current[first])) * 100.0;
		//float motor_throttle_iir = (float)motor_throttles[i]*0.9 +  (first*100 + temp)*0.1;
		ATOMIC_BLOCK(ATOMIC_FORCEON){
			g_Motor_Throttles[i] = (unsigned int)first*100 + (unsigned int)temp;
		}
	}
}

void Run_Motors(unsigned char setup_flag){
// ESC Interface - PPM (OneShot) control
// Inputs - Desired motor throttles (0-1000)
// Outputs - 400 Hz, 1-2 ms waveform to ESC
// Occurs within ISR, so don't need to worry about g_Motor_Throttles being changed while accessed
	// We want to map 0:1000 to 12000:24000 (1:2 ms)
	static unsigned int motor_lookup[1001] = {0};
	// Build the lookup table if it hasn't been built yet, enable pins for output
	if (setup_flag){ 
		for (unsigned int i=0;i<1001;i++){
			motor_lookup[i] = 12*i + 12000;
		}
		PORTD_DIR |= g_Motor1_Pin | g_Motor2_Pin | g_Motor3_Pin | g_Motor4_Pin; 
		return;
	}
	unsigned int mapped_throttle_commands[4] = {0};
	// Map commands, saturate if out of bounds
	for (unsigned char i=0;i<4;i++){
		g_Motor_Throttles[i] = (g_Motor_Throttles[i]>1000)?1000:g_Motor_Throttles[i];
		mapped_throttle_commands[i] = motor_lookup[g_Motor_Throttles[i]];
	}
	// Disable Timer
	TCA0_SINGLE_CTRLA &= ~TCA_SINGLE_ENABLE_bm;
	// Reset timer counts
	TCA0_SINGLE_CNT = 0;
	// Set motor throttles
	TCA0_SINGLE_CMP0 = mapped_throttle_commands[0]; // Motor 1, back
	TCA0_SINGLE_CMP1 = mapped_throttle_commands[1]; // Motor 2, left
	TCA0_SINGLE_CMP2 = mapped_throttle_commands[2]; // Motor 3, right
	// Set pins high
	PORTD_OUT |= g_Motor1_Pin | g_Motor2_Pin | g_Motor3_Pin;
	// Start Timers
	TCA0_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;
#if defined(AVR128DB48)
	TCA1_SINGLE_CMP0 = mapped_throttle_commands[3]; // Motor 4, front
	TCA1_SINGLE_CNT = 0;
	PORTD_OUT |= g_Motor4_Pin;
	TCA1_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;
#elif defined(AVR64DA28)
	TCB2_CNT = 0;
	TCB2_CCMP = mapped_throttle_commands[3]; // Motor 4, front
	PORTD_OUT |= g_Motor4_Pin;
	TCB2_CTRLA |= TCB_ENABLE_bm;
#endif
}

ISR(TCA0_CMP0_vect){
	// Set pin low
	PORTD_OUT &= ~g_Motor1_Pin;
	// Clear int flag
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm;
}

ISR(TCA0_CMP1_vect){
	// Set pin low
	PORTD_OUT &= ~g_Motor2_Pin;
	// Clear int flag
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP1_bm;
}

ISR(TCA0_CMP2_vect){
	// Set pin low
	PORTD_OUT &= ~g_Motor3_Pin;
	// Clear int flag
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP2_bm;
}
#if defined(AVR128DB48)
ISR(TCA1_CMP0_vect){
	// Set pin low
	PORTD_OUT &= ~g_Motor4_Pin;
	// Clear int flag
	TCA1_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm;
	// Only motor 4 uses TCA1, so disable timer
	TCA1_SINGLE_CTRLA &= ~TCA_SINGLE_ENABLE_bm;
}
#elif defined(AVR64DA28)
ISR(TCB2_INT_vect){
	// Set pin low
	PORTD_OUT &= ~g_Motor4_Pin;
	// Clear int flag
	TCB2_INTFLAGS = TCB_CAPT_bm;
	// Only motor 4 uses TCA1, so disable timer
	TCB2_CTRLA &= ~TCB_ENABLE_bm;
}
#endif