#include "Controllers.h"

//K_attitude = [1 0.2 0.2]; % P-I-D

float Height_MRAC(States &MCU, float h_ref){
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
    float k_r_dot = 0.1*h_ref*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);
    float w_dot = phi_x*(P_lyap[1]*e[0] + P_lyap[2]*e[1]);

    // Integrate controller gains
    k_x[0] += (k_x_dot[0]*d_t);
    k_x[1] += (k_x_dot[1]*d_t);
    k_r += (k_r_dot*d_t);
    w += (w_dot*d_t);

    // Update Thrust
    float temp = k_x[0]*h + k_x[1]*h_dot + k_r*h_ref - w*phi_x;
    float thrust = Saturate(temp, MAX_THRUST, 0.0);

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

    // float w_front = (omega_front > 0)?(sqrt(omega_front)):(0.0);
    // float w_right = (omega_right > 0)?(sqrt(omega_right)):(0.0);
    // float w_left = (omega_left > 0)?(sqrt(omega_left)):(0.0);
    // float w_back = (omega_back > 0)?(sqrt(omega_back)):(0.0);
    // Convert motor speeds in rad/s to throttle commands between 0-1000
    // Gain to convert rad/s to throttle command
    // omega = KT*i/k_t -> build mapping of i to throttle
    // Motor torque constant in N-m/A (1/KV)
    const float KT = 0.006366198;
    // Gain to convert (rad/s)^2 to A
    const float K = KT/k_t;
    // This array holds the current produced by the motor for each 10% of throttle, starting at 0%
    const float Current[11] = {0.0, 0.1, 0.7, 2.0, 4.1, 7.2, 10.9, 15.4, 20.5, 25.9, 31.8};
    uint8_t first;
    uint8_t last;
    uint8_t middle;
    float I;
    for (uint8_t i = 0; i < 4; i++){
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
        motor_throttles[i] = first*100 + (uint16_t)temp;
    }

    
    
    // const float K = 0.2344;
    // motor_throttles[0] = (uint16_t)(w_back*K);
    // motor_throttles[1] = (uint16_t)(w_left*K);
    // motor_throttles[2] = (uint16_t)(w_right*K);
    // motor_throttles[3] = (uint16_t)(w_front*K);
}

void Attitude_Gain_Lookup(uint8_t index, float Gains[3][6]){
    const float Gain_table[25][3][6] ={
    {{4.5308,    0.0000   , 1.1644 ,  -9.8481   ,-0.0000,   -1.7365}, // 0
    {0.1402  ,  4.3062   ,-1.5041   ,-0.5939   ,-9.3969  ,  3.3682},
    {-0.3852  ,  1.5673 ,   4.1325   , 1.6318 ,  -3.4202  , -9.2542}}, 

    {{4.5308   , 0.0000  ,  1.1644 ,  -9.8481 ,  -0.0000 ,  -1.7365}, // 1
    {0.0712  ,  4.5130  , -0.7637   ,-0.3015 ,  -9.8481   , 1.7101},
   {-0.4037   , 0.7958 ,   4.3309 ,   1.7101,   -1.7365  , -9.6985}},

    {{4.5308  ,  0.0000  ,  1.1644 ,  -9.8481 ,  -0.0000,   -1.7365}, // 2
   {-0.0000 ,   4.5826 ,  -0.0000  , -0.0000,  -10.0000 ,   0.0000},
   {-0.4099 ,  -0.0000 ,   4.3978 ,   1.7365 ,   0.0000 ,  -9.8481}},

    {{4.5308 ,  -0.0000 ,   1.1644 ,  -9.8481  ,  0.0000 ,  -1.7365}, // 3
   {-0.0712  ,  4.5130  ,  0.7637  ,  0.3015 ,  -9.8481 ,  -1.7101},
   {-0.4037 ,  -0.7958  ,  4.3309  ,  1.7101 ,   1.7365  , -9.6985}},

    {{4.5308 ,   0.0000 ,   1.1644  , -9.8481 ,  -0.0000 ,  -1.7365},
   {-0.1402 ,   4.3062  ,  1.5041,    0.5939 ,  -9.3969 ,  -3.3682},
   {-0.3852 ,  -1.5673 ,   4.1325 ,   1.6318  ,  3.4202 ,  -9.2542}},

    {{4.5695 ,  -0.0000 ,   0.5883,   -9.9619 ,   0.0000,   -0.8716}, // 5
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

    {{4.5826 ,  -0.0000  ,  0.0000,  -10.0000  ,  0.0000 ,  -0.0000}, // 10
   {-0.0000 ,   4.3062  , -1.5673 ,   0.0000 ,  -9.3969  ,  3.4202},
    {0.0000  ,  1.5673  ,  4.3062 ,  -0.0000 ,  -3.4202 ,  -9.3969}},

    {{4.5826 ,  -0.0000,   -0.0000 , -10.0000  ,  0.0000   , 0.0000},
   {-0.0000  ,  4.5130 ,  -0.7958 ,   0.0000,   -9.8481 ,   1.7365},
    {0.0000  ,  0.7958,    4.5130 ,   0.0000,   -1.7365 ,  -9.8481}},

    {{4.5826  ,  0.0000 ,  -0.0000  ,-10.0000  ,  0.0000  ,  0.0000}, // 12
    {0.0000  ,  4.5826 ,   0.0000 ,  -0.0000,  -10.0000 ,   0.0000},
    {0.0000  , -0.0000 ,   4.5826 ,  -0.0000   , 0.0000 , -10.0000}},

   {{4.5826 ,  -0.0000 ,  -0.0000 , -10.0000,    0.0000  ,  0.0000}, // 13
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
        { 0, 1, 2, 3, 4},
        { 5, 6, 7, 8, 9},
        {10,11,12,13,14},
        {15,16,17,18,19},
        {20,21,22,23,24}
    };
    const float d_t = 0.0025;
    float K[3][6];

    const int8_t theta[5] = {-20, -10, 0, 10, 20};
    const int8_t phi[5] = {-20, -10, 0, 10, 20};
    float theta_deg = MCU.Euler[1]*R2D;
    int16_t theta_m = (int16_t)theta_deg;
    float phi_deg = MCU.Euler[0]*R2D;
    int16_t phi_m = (int16_t)phi_deg;

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
    float temp;
    for (uint8_t i = 0; i < 3; i++){
        Euler_error[i] = Angle_difference(Reference.Euler[i], MCU.Euler[i]); //Reference.Euler[i]-MCU.Euler[i];
        Euler_error_int[i] += (Euler_error[i]*d_t);
    }
    for (uint8_t i = 0; i < 3; i++){
        temp = -(MCU.Euler[0]*K[i][0] + MCU.Euler[1]*K[i][1] + Angle_Discontinuity(MCU.Euler[2])*K[i][2]
         + Euler_error_int[0]*K[i][3] + Euler_error_int[1]*K[i][4] + Euler_error_int[2]*K[i][5]);
        Reference.w[i] = 0.1*Saturate(temp, MAX_W, -MAX_W);
    }
}

void Angular_Rate_Control(States &MCU, States &Reference, float desired_moments[3]){
    // A PID controller is used to control each body angular rate and set moments
    const float d_t = 0.01;
    const float K_p = 0.1;
    const float K_i = 0.02;
    const float K_d = 0.02;
    static float e_last[3];
    static float e_int[3];
    for (uint8_t i = 0; i < 3; i++){
        float e = Reference.w[i] - MCU.w[i];
        float e_d = e - e_last[i];
        e_last[i] = e;
        e_int[i] += (e*d_t);
        float temp = K_p*e + K_i*e_int[i] + K_d*e_d;
        desired_moments[i] = Saturate(temp, MAX_MOMENT, -MAX_MOMENT);
    }
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
    if (diff > PI){
        diff -= 2.0*PI;
    }
    else if (diff < -PI){
        diff += 2.0*PI;
    }
    return diff;
}

float Angle_Discontinuity(float angle){
    return (angle > PI)?(angle - 2.0*PI):(angle);
}