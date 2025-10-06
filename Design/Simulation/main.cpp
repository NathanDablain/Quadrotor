#include "Quadrotor.h"
#include "Sim_Time.h"
#include "Gaussian.h"

void Initialize_Monte_Carlo_Data(Monte_Carlo_Data *MC_Data, uint32_t mc_seed, uint32_t number_of_runs);

int main(){
    Sim_Time Sim_time_step = {.Seconds = 0, .MicroSeconds = 100};
    Sim_Time Sim_finish_time = {.Seconds = 150, .MicroSeconds = 0};

    uint32_t mc_seed = 191;
    uint32_t number_of_runs = 1;
    Monte_Carlo_Data MC_Data[number_of_runs];
    Initialize_Monte_Carlo_Data(MC_Data, mc_seed, number_of_runs);
    uint32_t crashes = 0;
    uint32_t fails = 0;
    uint32_t landings = 0;

    for (uint32_t i = 0; i < number_of_runs; i++){
        Quadrotor Drone(Sim_time_step, Sim_finish_time);
        Drone.Set_Monte_Carlo_Data(MC_Data[i]);
        Drone.Run_sim();

        if (Drone.PIC.Flight_Controller_Status == Crashed_p32){
            cout << "Run " << mc_seed + i << " ended in crash" << endl;
            crashes++;
        }
        else if (Drone.PIC.Flight_Controller_Status == Standby_p32){
            cout << "Run " << mc_seed + i << " landed successfully at " << Drone.sim_t.Time_fp() << endl;
            landings++;
        }
        else {
            cout << "Run " << mc_seed + i << " was unsuccessful" << endl;
            fails++;
        }
    
    }
    if (crashes > 0){
        cout << crashes << "/" << number_of_runs << " crashed" << endl;
    }
    if (fails > 0){
         cout << fails << "/" << number_of_runs << " failed to take off and land" << endl;
    }
    if (landings > 0){
        cout << landings << "/" << number_of_runs << " landed successfully" << endl;
    }
    return 0;
}

void Initialize_Monte_Carlo_Data(Monte_Carlo_Data *MC_Data, uint32_t mc_seed, uint32_t number_of_runs){
    const double mass_mean = 0.5885;
    const double mass_variance = mass_mean/10.0;
    // Distance from front and back motor thrust vectors to drone center of gravity in (m)
    const double length_f_b_mean =  0.127;
    const double length_f_b_variance = length_f_b_mean/10.0;
    // Distance from left and right motor thrust vectors to drone center of gravity in (m)
    const double length_l_r_mean = 0.125;
    const double length_l_r_variance = length_l_r_mean/10.0;
    // Actual mass moments of drone in (kg-m^2)
    const double Ixx_mean = 0.0018;
    const double Ixx_variance = Ixx_mean/6.0;
    const double Iyy_mean = 0.00356;
    const double Iyy_variance = Iyy_mean/6.0;
    const double Izz_mean = 0.00208;
    const double Izz_variance = Izz_mean/6.0;
    const double Ixy_mean = 0.0000227;
    const double Ixy_variance = Ixy_mean/2.0;
    const double Ixz_mean = -0.0000012;
    const double Ixz_variance = Ixz_mean/2.0;
    const double Iyz_mean = 0.0000010;
    const double Iyz_variance = Iyz_mean/2.0;
    // Motor parameters
    const double Motor_deadzone_mean = 30.0;
    const double Motor_deadzone_variance = 10.0;
    const double Motor_slope_mean = 228.0;
    const double Motor_slope_variance = Motor_slope_mean/20.0;
    const double Motor_zero_offset_mean = 316.0;
    const double Motor_zero_offset_variance = Motor_zero_offset_mean/20.0;
    // Propeller parameters
    const double Propeller_Force_Constant_mean = 0.000001;
    const double Propeller_Force_Constant_variance = Propeller_Force_Constant_mean/10.0;
    const double Propeller_Torque_Constant_mean = 0.000000011;
    const double Propeller_Torque_Constant_variance = Propeller_Torque_Constant_mean/5.0;
    const double Propeller_mu_mean = 1.0;
    const double Propeller_mu_variance = 0.5;
    // Initial conditions
    // Here variances are in degrees
    const double Initial_roll_variance = 7.5;
    const double Initial_pitch_variance = 7.5;
    const double Initial_yaw_variance = 180.0;
    const double Initial_roll_mean = 0.0;
    const double Initial_pitch_mean = 0.0;
    const double Initial_yaw_mean = 0.0;

    Monte_Carlo_Data Local_Data[number_of_runs+mc_seed];
    
    Gaussian gaus_mass(mass_variance, mass_mean);
    Gaussian gaus_length_f_b(length_f_b_variance, length_f_b_mean);
    Gaussian gaus_length_l_r(length_l_r_variance, length_l_r_mean);
    Gaussian gaus_inertia_Ixx(Ixx_variance, Ixx_mean);
    Gaussian gaus_inertia_Iyy(Iyy_variance, Iyy_mean);
    Gaussian gaus_inertia_Izz(Izz_variance, Izz_mean);
    Gaussian gaus_inertia_Ixy(Ixy_variance, Ixy_mean);
    Gaussian gaus_inertia_Ixz(Ixz_variance, Ixz_mean);
    Gaussian gaus_inertia_Iyz(Iyz_variance, Iyz_mean);
    Gaussian gaus_motor_deadzone1(Motor_deadzone_variance, Motor_deadzone_mean);
    Gaussian gaus_motor_deadzone2(Motor_deadzone_variance, Motor_deadzone_mean);
    Gaussian gaus_motor_deadzone3(Motor_deadzone_variance, Motor_deadzone_mean);
    Gaussian gaus_motor_deadzone4(Motor_deadzone_variance, Motor_deadzone_mean);
    Gaussian gaus_motor_slope(Motor_slope_variance, Motor_slope_mean);
    Gaussian gaus_motor_zero_offset(Motor_zero_offset_variance, Motor_zero_offset_mean);
    Gaussian gaus_propeller_mu(Propeller_mu_variance, Propeller_mu_mean);
    Gaussian gaus_propeller_force_constant(Propeller_Force_Constant_variance, Propeller_Force_Constant_mean);
    Gaussian gaus_propeller_torque_constant(Propeller_Torque_Constant_variance, Propeller_Torque_Constant_mean);
    Gaussian gaus_initial_roll(Initial_roll_variance, Initial_roll_mean);
    Gaussian gaus_initial_pitch(Initial_pitch_variance, Initial_pitch_mean);
    Gaussian gaus_initial_yaw(Initial_yaw_variance, Initial_yaw_mean);

    uint32_t MC_Data_counter = 0;
    for (uint32_t i = 0; i < mc_seed + number_of_runs; i++){
        Local_Data[i].mass = gaus_mass.Get_val();
        Local_Data[i].length_f_b = gaus_length_f_b.Get_val();
        Local_Data[i].length_l_r = gaus_length_l_r.Get_val();
        Local_Data[i].interia_xx = gaus_inertia_Ixx.Get_val();
        Local_Data[i].inertia_yy = gaus_inertia_Iyy.Get_val();
        Local_Data[i].inertia_zz = gaus_inertia_Izz.Get_val();
        Local_Data[i].inertia_xy = gaus_inertia_Ixy.Get_val();
        Local_Data[i].inertia_xz = gaus_inertia_Ixz.Get_val();
        Local_Data[i].inertia_yz = gaus_inertia_Iyz.Get_val();
        Local_Data[i].Motor_deadzone[0] = static_cast<uint16_t>(fabs(gaus_motor_deadzone1.Get_val()));
        Local_Data[i].Motor_deadzone[1] = static_cast<uint16_t>(fabs(gaus_motor_deadzone2.Get_val()));
        Local_Data[i].Motor_deadzone[2] = static_cast<uint16_t>(fabs(gaus_motor_deadzone3.Get_val()));
        Local_Data[i].Motor_deadzone[3] = static_cast<uint16_t>(fabs(gaus_motor_deadzone4.Get_val()));
        Local_Data[i].Motor_zero_offset = gaus_motor_zero_offset.Get_val();
        Local_Data[i].Motor_slope = gaus_motor_slope.Get_val();
        Local_Data[i].Propeller_force_constant = gaus_propeller_force_constant.Get_val();
        Local_Data[i].Propeller_torque_constant = gaus_propeller_torque_constant.Get_val();
        Local_Data[i].Propeller_mu = fabs(gaus_propeller_mu.Get_val());
        Local_Data[i].Initial_roll = gaus_initial_roll.Get_val();
        Local_Data[i].Initial_pitch = gaus_initial_pitch.Get_val();
        Local_Data[i].Initial_yaw = gaus_initial_yaw.Get_val();
        if (i >= mc_seed){
            MC_Data[MC_Data_counter] = Local_Data[i];
            MC_Data_counter++;
        }
    }
}
