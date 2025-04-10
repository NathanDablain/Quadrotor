#include "Controllers.h"

float Height_MRAC(States *Drone, float h_ref){
// Altitude Controller - Adaptive Direct MRAC
// Inputs - Desired NED drone positions
// Outputs - Desired thrust
    const float d_t = 0.25;
    // Reference Model
    // h and h_dot
    static float x_ref[2];
    float x_ref_dot[2];
    // Natural frequency and damping ratio for reference model
    const float w_n = 1.0;
    const float zeta = 0.707;
    const float c1 = 2.0*zeta*w_n;
    const float c2 = pow(w_n, 2);
    float h = -Drone->Position_NED[2];
    
    x_ref_dot[0] = x_ref[1];
    x_ref_dot[1] = -c1*x_ref[0] - c2*x_ref[1] + c1*h_ref;
    x_ref[0] += (x_ref_dot[0]*d_t);
    x_ref[1] += (x_ref_dot[1]*d_t);

    // Update Gains
    static float h_last;
    float e[2];
    e[0] = x_ref[0] - h;
    float h_dot = (h - h_last)/d_t;
    h_last = h;
    e[1] = x_ref[1] - h_dot;

    // This is actually a 2x2 matrix but P_12 = P_21 so only store that value once as P_lyap[1]
    static float k_x[2], k_r, w;
    const float P_lyap[3] = {36.2143, -0.5, 50.7070};
    const float gamma_x[2] = {0.2, 10.0};
    float k_x_dot[2];

    float phi_x = cos(Drone->Euler[0])*cos(Drone->Euler[1]);
    k_x_dot[0] = gamma_x[0]*h*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);
    k_x_dot[1] = gamma_x[1]*h_dot*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);
    float k_r_dot = h_ref*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);
    float w_dot = phi_x*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);

    // Integrate controller gains
    k_x[0] += (k_x_dot[0]*d_t);
    k_x[1] += (k_x_dot[1]*d_t);
    k_r += (k_r_dot*d_t);
    w += (w_dot*d_t);

    // Update Thrust
    float thrust = k_x[0]*h + k_x[1]*h_dot + k_r*h_ref - w*phi_x;
    if (thrust > MAX_THRUST){
        thrust = MAX_THRUST;
    }
    else if (thrust < 0.0){
        thrust = 0.0;
    }
    return thrust;
}

void Attitude_LQR(States *Drone, States *Reference){
// Attitude Controller - Linearized LQR w/integrator
// Inputs - Desired Euler angles
// Outputs - Desired body angular rates
	const unsigned char index_map[5][5] = {
		{ 0, 1, 2, 3, 4},
		{ 5, 6, 7, 8, 9},
		{10,11,12,13,14},
		{15,16,17,18,19},
		{20,21,22,23,24}
	};
	const float d_t = 0.0025;
	float K[3][6];

	const signed char theta[5] = {-20, -10, 0, 10, 20};
	const signed char phi[5] = {-20, -10, 0, 10, 20};
	float theta_deg = Drone->Euler[1]*R2D;
	signed int theta_m = (signed int)theta_deg;
	float phi_deg = Drone->Euler[0]*R2D;
	signed int phi_m = (signed int)phi_deg;

	unsigned char theta_index = INDEX_NOT_SET;
	unsigned char phi_index = INDEX_NOT_SET;
	for (unsigned char i = 0; i<sizeof(theta); i++){
		if (abs(theta_m-theta[i]) <= 5){
			theta_index = i;
			break;
		}
	}
	if (theta_index == INDEX_NOT_SET){
		theta_index = (theta_m < theta[0])?(0):(sizeof(theta) - 1);
	}

	for (unsigned char i = 0; i<sizeof(phi); i++){
		if (abs(phi_m-phi[i]) <= 5){
			phi_index = i;
			break;
		}
	}
	if (phi_index == INDEX_NOT_SET){
		phi_index = (phi_m < phi[0])?(0):(sizeof(phi) - 1);
	}
	unsigned char Gain_index = index_map[theta_index][phi_index];
	Attitude_Gain_Lookup(Gain_index, K);
	// LQR includes integrator in addition to state feedback
	static float Euler_error_int[3];
	float Euler_error[3];
	float temp;
	for (unsigned char i = 0; i < 3; i++){
		Euler_error[i] = Angle_difference(Reference->Euler[i], Drone->Euler[i]);
		Euler_error_int[i] += (Euler_error[i]*d_t);
	}
	for (unsigned char i = 0; i < 3; i++){
		temp = -(Drone->Euler[0]*K[i][0] + Drone->Euler[1]*K[i][1] + Angle_Discontinuity(Drone->Euler[2])*K[i][2]
		+ Euler_error_int[0]*K[i][3] + Euler_error_int[1]*K[i][4] + Euler_error_int[2]*K[i][5]);
		Reference->w[i] = 0.1*Saturate(temp, MAX_W, -MAX_W);
	}
}

void Attitude_Gain_Lookup(unsigned char index, float Gains[3][6]){
    const float Gain_table[25][3][6] ={
    {{4.5308,    0.0000   , 1.1644 ,  -9.8481   ,-0.0000,   -1.7365},
    {0.1402  ,  4.3062   ,-1.5041   ,-0.5939   ,-9.3969  ,  3.3682},
    {-0.3852  ,  1.5673 ,   4.1325   , 1.6318 ,  -3.4202  , -9.2542}},

    {{4.5308   , 0.0000  ,  1.1644 ,  -9.8481 ,  -0.0000 ,  -1.7365},
    {0.0712  ,  4.5130  , -0.7637   ,-0.3015 ,  -9.8481   , 1.7101},
   {-0.4037   , 0.7958 ,   4.3309 ,   1.7101,   -1.7365  , -9.6985}},

    {{4.5308  ,  0.0000  ,  1.1644 ,  -9.8481 ,  -0.0000,   -1.7365},
   {-0.0000 ,   4.5826 ,  -0.0000  , -0.0000,  -10.0000 ,   0.0000},
   {-0.4099 ,  -0.0000 ,   4.3978 ,   1.7365 ,   0.0000 ,  -9.8481}},

    {{4.5308 ,  -0.0000 ,   1.1644 ,  -9.8481  ,  0.0000 ,  -1.7365},
   {-0.0712  ,  4.5130  ,  0.7637  ,  0.3015 ,  -9.8481 ,  -1.7101},
   {-0.4037 ,  -0.7958  ,  4.3309  ,  1.7101 ,   1.7365  , -9.6985}},

    {{4.5308 ,   0.0000 ,   1.1644  , -9.8481 ,  -0.0000 ,  -1.7365},
   {-0.1402 ,   4.3062  ,  1.5041,    0.5939 ,  -9.3969 ,  -3.3682},
   {-0.3852 ,  -1.5673 ,   4.1325 ,   1.6318  ,  3.4202 ,  -9.2542}},

    {{4.5695 ,  -0.0000 ,   0.5883,   -9.9619 ,   0.0000,   -0.8716},
    {0.0713  ,  4.3062  , -1.5515  , -0.2981  , -9.3969 ,   3.4072},
   {-0.1958 ,   1.5673 ,   4.2627 ,   0.8190 ,  -3.4202 ,  -9.3612}},

    {{4.5695  , -0.0000  ,  0.5883 ,  -9.9619 ,   0.0000,   -0.8716},
    {0.0362 ,   4.5130 ,  -0.7877,   -0.1513  , -9.8481 ,   1.7299},
   {-0.2052  ,  0.7958  ,  4.4674 ,   0.8583 ,  -1.7365 ,  -9.8106}},

    {{4.5695 ,  -0.0000 ,   0.5883,   -9.9619 ,  -0.0000,   -0.8716},
    {0.0000  ,  4.5826  ,  0.0000  , -0.0000 , -10.0000  , -0.0000},
   {-0.2084  , -0.0000  ,  4.5363   , 0.8716,    0.0000   ,-9.9619}},

    {{4.5695,    0.0000 ,   0.5883 ,  -9.9619 ,  -0.0000  , -0.8716},
   {-0.0362  ,  4.5130 ,   0.7877 ,   0.1513 ,  -9.8481  , -1.7299},
   {-0.2052   ,-0.7958,    4.4674,    0.8583,    1.7365 ,  -9.8106}},

    {{4.5695 ,   0.0000 ,   0.5883  , -9.9619 ,  -0.0000,   -0.8716},
   {-0.0713 ,   4.3062 ,   1.5515 ,   0.2981 ,  -9.3969 ,  -3.4072},
   {-0.1958 ,  -1.5673 ,   4.2627  ,  0.8190 ,   3.4202 ,  -9.3612}},

    {{4.5826 ,  -0.0000  ,  0.0000,  -10.0000  ,  0.0000 ,  -0.0000},
   {-0.0000 ,   4.3062  , -1.5673 ,   0.0000 ,  -9.3969  ,  3.4202},
    {0.0000  ,  1.5673  ,  4.3062 ,  -0.0000 ,  -3.4202 ,  -9.3969}},

    {{4.5826 ,  -0.0000,   -0.0000 , -10.0000  ,  0.0000   , 0.0000},
   {-0.0000  ,  4.5130 ,  -0.7958 ,   0.0000,   -9.8481 ,   1.7365},
    {0.0000  ,  0.7958,    4.5130 ,   0.0000,   -1.7365 ,  -9.8481}},

    {{4.5826  ,  0.0000 ,  -0.0000  ,-10.0000  ,  0.0000  ,  0.0000},
    {0.0000  ,  4.5826 ,   0.0000 ,  -0.0000,  -10.0000 ,   0.0000},
    {0.0000  , -0.0000 ,   4.5826 ,  -0.0000   , 0.0000 , -10.0000}},

   {{4.5826 ,  -0.0000 ,  -0.0000 , -10.0000,    0.0000  ,  0.0000},
   {-0.0000 ,   4.5130,    0.7958 ,   0.0000,   -9.8481 ,  -1.7365},
    {0.0000  , -0.7958,    4.5130 ,  -0.0000 ,   1.7365 ,  -9.8481}},

    {{4.5826 ,  -0.0000 ,  -0.0000 , -10.0000 ,   0.0000 ,   0.0000},
   {-0.0000  ,  4.3062 ,   1.5673 ,   0.0000 ,  -9.3969 ,  -3.4202},
    {0.0000 ,  -1.5673 ,   4.3062  ,  0.0000  ,  3.4202 ,  -9.3969}},

    {{4.5695 ,   0.0000  , -0.5883 ,  -9.9619  , -0.0000 ,   0.8716},
   {-0.0713,    4.3062 ,  -1.5515 ,   0.2981 ,  -9.3969 ,   3.4072},
    {0.1958  ,  1.5673  ,  4.2627 ,  -0.8190 ,  -3.4202 ,  -9.3612}},

    {{4.5695,   -0.0000  , -0.5883   ,-9.9619 ,   0.0000  ,  0.8716},
   {-0.0362  ,  4.5130 ,  -0.7877  ,  0.1513 ,  -9.8481  ,  1.7299},
    {0.2052 ,   0.7958  ,  4.4674 ,  -0.8583  , -1.7365 ,  -9.8106}},

    {{4.5695  ,  0.0000 ,  -0.5883 ,  -9.9619 ,  -0.0000 ,   0.8716},
   {-0.0000  ,  4.5826 ,  -0.0000 ,  -0.0000,  -10.0000 ,  -0.0000},
    {0.2084  ,  0.0000 ,   4.5363 ,  -0.8716 ,   0.0000 ,  -9.9619}},

    {{4.5695,    0.0000  , -0.5883,   -9.9619 ,  -0.0000,    0.8716},
    {0.0362  ,  4.5130  ,  0.7877  , -0.1513 ,  -9.8481  , -1.7299},
    {0.2052   ,-0.7958 ,   4.4674   ,-0.8583,    1.7365  , -9.8106}},

    {{4.5695 ,  -0.0000 ,  -0.5883 ,  -9.9619 ,   0.0000 ,   0.8716},
    {0.0713  ,  4.3062  ,  1.5515  , -0.2981 ,  -9.3969  , -3.4072},
    {0.1958 ,  -1.5673  ,  4.2627  , -0.8190 ,   3.4202  , -9.3612}},

    {{4.5308 ,   0.0000,   -1.1644 ,  -9.8481,   -0.0000  ,  1.7365},
   {-0.1402  ,  4.3062 ,  -1.5041 ,   0.5939 ,  -9.3969  ,  3.3682},
    {0.3852  ,  1.5673 ,   4.1325 ,  -1.6318  , -3.4202  , -9.2542}},

    {{4.5308 ,  -0.0000 ,  -1.1644 ,  -9.8481,   -0.0000 ,   1.7365},
   {-0.0712  ,  4.5130 ,  -0.7637 ,   0.3015 ,  -9.8481 ,   1.7101},
    {0.4037  ,  0.7958 ,   4.3309  , -1.7101 ,  -1.7365 ,  -9.6985}},

    {{4.5308  ,  0.0000 ,  -1.1644 ,  -9.8481,   -0.0000  ,  1.7365},
   {-0.0000 ,   4.5826 ,   0.0000 ,  -0.0000 , -10.0000 ,  -0.0000},
    {0.4099  ,  0.0000 ,   4.3978 ,  -1.7365 ,   0.0000,   -9.8481}},

    {{4.5308 ,  -0.0000 ,  -1.1644 ,  -9.8481 ,   0.0000 ,   1.7365},
    {0.0712 ,   4.5130  ,  0.7637 ,  -0.3015,   -9.8481 ,  -1.7101},
    {0.4037 ,  -0.7958  ,  4.3309 ,  -1.7101 ,   1.7365 ,  -9.6985}},

    {{4.5308 ,  -0.0000,   -1.1644 ,  -9.8481 ,   0.0000 ,   1.7365},
    {0.1402 ,   4.3062 ,   1.5041 ,  -0.5939 ,  -9.3969 ,  -3.3682},
    {0.3852 ,  -1.5673 ,   4.1325 ,  -1.6318  ,  3.4202,   -9.2542}}};
    
    for (unsigned char i = 0; i<3; i++){
        for (unsigned char j = 0; j<6; j++){
            Gains[i][j] = Gain_table[index][i][j];
        }
    }
}

void Angular_Rate_Control(States *Drone, States *Reference, float desired_moments[3]){
// Orientation Controller - PID
// Inputs - Desired body angular rates
// Outputs - Desired body torques
 // A PID controller is used to control each body angular rate and set moments
 //const float d_t = 0.01;
 const float K_p = 0.05;
 //const float K_i = 0.02;
 //const float K_d = 0.02;
 //static float e_last[3];
 //static float e_int[3];
 for (unsigned char i = 0; i < 3; i++){
	 float e = Reference->w[i] - Drone->w[i];
	 //float e_d = e - e_last[i];
	 //e_last[i] = e;
	 //e_int[i] += (e*d_t);
	 //float temp = K_p*e + K_i*e_int[i] + K_d*e_d;
	 float temp = K_p*e;
	 desired_moments[i] = Saturate(temp, MAX_MOMENT, -MAX_MOMENT);
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
		motor_throttles[i] = first*100 + (unsigned int)temp;
	}
}

void Run_Motors(unsigned int Throttle_Commands[4]){
// ESC Interface - PPM (OneShot) control
// Inputs - Desired motor throttles (0-100)
// Outputs - 100 Hz, 1-2us waveform to ESC
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

float Saturate(float value_in, float max_value, float min_value){
	if (value_in > max_value){
		return max_value;
	}
	else if (value_in < min_value){
		return min_value;
	}
	return value_in;
}

float Angle_difference(float angle1, float angle2){
	float diff = angle1 - angle2;
	if (diff > M_PI){
		diff -= 2.0*M_PI;
	}
	else if (diff < -M_PI){
		diff += 2.0*M_PI;
	}
	return diff;
}

float Angle_Discontinuity(float angle){
	return (angle > M_PI)?(angle - 2.0*M_PI):(angle);
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