#include "main.h"
// Altitude Controller - Adaptive Direct MRAC
// Inputs - Desired NED drone positions
// Outputs - Desired thrust
float MRAC_Altitude(States *Drone, States *Reference){
	static float height_last = 0;
	static float x_ref[2] = {0};
	//const float d_t = 
	// Run Reference Model
	
	// Update controller gains
	
	// Calculate thrust output
	
	return 0.0;
}
// Attitude Controller - Linearized LQR w/integrator
// Inputs - Desired Euler angles
// Outputs - Desired body angular rates

// Orientation Controller - PID
// Inputs - Desired body angular rates
// Outputs - Desired body torques

// Motor mixer
// Inputs - Desired thrust and body torques
// Outputs - Desired throttle command on each of 4 BLDC motors 

// ESC Interface - PPM (OneShot) control
// Inputs - Desired motor throttles (0-100)
// Outputs - 100 Hz, 1-2us waveform to ESC 
void Run_Motors(unsigned int Throttle_Commands[4]){
	// We want to map 0:1000 to 3000:6000 (1000:2000 us)
	static unsigned int motor_lookup[1001] = {0};
	// Build the lookup table if it hasn't been built yet, enable pins for output
	if (!(motor_lookup[0])){ 
		for (unsigned int i=0;i<1001;i++){
			motor_lookup[i] = 3*i + 3000;
		}
		PORTD_DIR |= PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm; 
	}
	unsigned int mapped_throttle_commands[4] = {0};
	// Map commands, saturate if out of bounds
	for (unsigned char i=0;i<4;i++){
		Throttle_Commands[i] = (Throttle_Commands[i]>100)?100:Throttle_Commands[i];
		mapped_throttle_commands[i] = motor_lookup[Throttle_Commands[i]];
	}
	// Set motor throttles
	TCA0_SINGLE_CMP0 = mapped_throttle_commands[0]; // Motor 1, back
	//TCA0_SINGLE_CMP1 = mapped_throttle_commands[1]; // Motor 2, left
	//TCA0_SINGLE_CMP2 = mapped_throttle_commands[2]; // Motor 3, front
	//TCA1_SINGLE_CMP0 = mapped_throttle_commands[3]; // Motor 4, right
	// Reset timer counts
	TCA0_SINGLE_CNT = 0;
	//TCA1_SINGLE_CNT = 0;
	// Set pins high
	PORTD_OUT |= PIN0_bm; // | PIN1_bm | PIN2_bm | PIN3_bm;
	// Start Timers
	TCA0_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;
	// TCA1_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;
}

ISR(TCA0_CMP0_vect){
	// Set pin low
	PORTD_OUT &= ~PIN0_bm;
	// Clear int flag
	TCA0_SINGLE_INTFLAGS |= TCA_SINGLE_CMP0_bm;
	// Disable timer
	TCA0_SINGLE_CTRLA &= ~TCA_SINGLE_ENABLE_bm;
}