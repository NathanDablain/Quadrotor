#include "Controllers.h"
//K_attitude = [1 0.2 0.2]; % P-I-D

float Height_MRAC(States &MCU, float h_ref){
    const float d_t = 0.025;
    // Reference Model
    // h and h_dot
    static float x_ref[2];
    float x_ref_dot[2];
    // Natural frequency and damping ratio for reference model
    const float w_n = 1.0;
    const float zeta = 0.707;
    const float c1 = 2.0*zeta*w_n;
    const float c2 = pow(w_n, 2);
    float h = -MCU.Position_NED[2];
    
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

    float phi_x = cos(MCU.Euler[0])*cos(MCU.Euler[1]);
    k_x_dot[0] = gamma_x[0]*-MCU.Position_NED[2]*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);
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

void Set_throttles(uint16_t motor_throttles[4], float desired_thrust, float desired_moments[3]){
    // -> Back motor (0) produces negative pitching torque and negative yawing torque
    // -> Left motor (1) produces positive rolling torque and positive yawing torque
    // -> Right motor (2) produces negative rolling torque and positive yawing torque
    // -> Front motor (3) produces positive pitching torque and negative yawing torque
    // Propeller thrust constant in N/(rad/s)^2 
    const float k_f = 0.000001;
    // Propeller torque constant in N-m/(rad/s)^2
    const float k_t = 0.000000011;
    // Distance from front and back motor thrust vectors to drone center of gravity in (m)
    const float length_f_b =  0.117;
    // Distance from left and right motor thrust vectors to drone center of gravity in (m)
    const float length_l_r = 0.1205;
    const float c2 = k_t*length_f_b;
    const float c3 = k_t*length_l_r;
    const float denom_1 = (2.0*k_f*length_f_b*(c2 + c3));
    const float denom_2 = (2.0*k_f*length_l_r*(c2 + c3));
    const float c1 = c2*length_l_r;
    const float c4 = k_f*length_f_b;
    const float c5 = k_f*length_l_r;

    float omega_front = (desired_moments[1]*c2 - desired_moments[2]*c4 + desired_moments[1]*c3 + desired_thrust*c1)/denom_1;
    float omega_right = -(desired_moments[0]*c2 - desired_moments[2]*c5 + desired_moments[0]*c3 - desired_thrust*c1)/denom_2;
    float omega_left = (desired_moments[0]*c2 + desired_moments[2]*c5 + desired_moments[0]*c3 + desired_thrust*c1)/denom_2;
    float omega_back = -(desired_moments[2]*c4 + desired_moments[1]*c2 + desired_moments[1]*c3 - desired_thrust*c1)/denom_1;

    float w_front = (omega_front > 0)?(sqrt(omega_front)):(0.0);
    float w_right = (omega_right > 0)?(sqrt(omega_right)):(0.0);
    float w_left = (omega_left > 0)?(sqrt(omega_left)):(0.0);
    float w_back = (omega_back > 0)?(sqrt(omega_back)):(0.0);
    // Convert motor speeds in rad/s to throttle commands between 0-1000
    // Gain to convert rad/s to throttle command
    const float K = 0.2344;
    motor_throttles[0] = (uint16_t)(w_back*K);
    motor_throttles[1] = (uint16_t)(w_left*K);
    motor_throttles[2] = (uint16_t)(w_right*K);
    motor_throttles[3] = (uint16_t)(w_front*K);
}

void Attitude_Gain_Lookup(uint8_t index, float Gains[3][6]){
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

    for (uint8_t i = 0; i<3; i++){
        for (uint8_t j = 0; j<6; j++){
            Gains[i][j] = Gain_table[index][i][j];
        }
    }
}

void Attitude_LQR(States &MCU, States &Reference){
    const uint8_t index_map[5][5] = {
        { 1, 2, 3, 4, 5},
        { 6, 7, 8, 9,10},
        {11,12,13,14,15},
        {16,17,18,19,20},
        {21,22,23,24,25}
    };
    const float d_t = 0.025;
    float K[3][6];

    const int8_t theta[5] = {-20, -10, 0, 10, 20};
    const int8_t phi[5] = {-20, -10, 0, 10, 20};
    int16_t theta_m = (int16_t)((MCU.Euler[1]*R2D));
    int16_t phi_m = (int16_t)((MCU.Euler[0]*R2D));

    uint8_t theta_index = INDEX_NOT_SET;
    uint8_t phi_index = INDEX_NOT_SET;
    for (uint8_t i = 0; i<sizeof(theta); i++){
        if (abs(theta_m-theta[i]) <= 5){
            theta_index = i;
            break;
        }
    }
    if (theta_index == INDEX_NOT_SET){
        if (theta_m < theta[0]){
            theta_index = 0;
        }
        else{
            theta_index = sizeof(theta) - 1;
        }
    }

    for (uint8_t i = 0; i<sizeof(phi); i++){
        if (abs(phi_m-phi[i]) <= 5){
            phi_index = i;
            break;
        }
    }
    if (phi_index == INDEX_NOT_SET){
        if (phi_m < phi[0]){
            phi_index = 0;
        }
        else{
            phi_index = sizeof(phi) - 1;
        }
    }
    uint8_t Gain_index = index_map[theta_index][phi_index];
    Attitude_Gain_Lookup(Gain_index, K);
    // LQR includes integrator in addition to state feedback
    static float Euler_error_int[3];
    float Euler_error[3];
    for (uint8_t i = 0; i < 3; i++){
        Euler_error[i] = Reference.Euler[i]-MCU.Euler[i];
        Euler_error_int[i] += (Euler_error[i]*d_t);
    }
    for (uint8_t i = 0; i < 3; i++){
        Reference.w[i] = -1*(MCU.Euler[0]*K[i][0] + MCU.Euler[1]*K[i][1] + MCU.Euler[2]*K[i][2]
         + Euler_error_int[0]*K[i][3] + Euler_error_int[1]*K[i][4] + Euler_error_int[2]*K[i][5]);
    }
}

void Angular_Rate_Control(States &MCU, States &Reference, float desired_moments[3]){
    // A PID controller is used to control each body angular rate and set moments
    const float d_t = 0.01;
    const float K_p = 1.0;
    const float K_i = 0.3;
    const float K_d = 0.2;
    static float e_last[3];
    static float e_int[3];
    for (uint8_t i = 0; i < 3; i++){
        float e = Reference.w[i] - MCU.w[i];
        float e_d = e - e_last[i];
        e_last[i] = e;
        e_int[i] += (e*d_t);
        desired_moments[i] = K_p*e + K_i*e_int[i] + K_d*e_d;
    }
}