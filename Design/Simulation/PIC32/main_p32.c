#include <stdio.h>
#include <stdbool.h>
#include "External_Interface.h"
#include "Barometer_p32.h"
#include "IMU_p32.h"
#include "Magnetometer_p32.h"
#include "Lora_p32.h"
#include "Guidance_p32.h"
#include "Navigation_p32.h"
#include "Controllers_p32.h"
#include "Global_Variables_p32.h"

static bool local_log_flag;
void Write_Output(bool initialize);

void Initialize_p32(bool log_flag){
    local_log_flag = log_flag;

    Initialize_Bar();

    Initialize_IMU();
    
    Initialize_Mag();

    Initialize_LORA();

    Manage_FC_Status(Standby);
    
    Initialize_Guidance_Machine();
    
    Initialize_Controllers();
}

void Execute_p32(){
    g_seconds = e_Current_Time.seconds;
    
    Run_Barometer_Machine();

    Run_IMU_Machine();
    
    Run_Magnetometer_Machine();

    Run_LORA();
    
    if (g_Flight_Controller_Status == System_Calibration){
        Run_Ground_Filter(false);
    }
    else if (g_Flight_Controller_Status == Flying || g_Flight_Controller_Status == Landing){
        Guidance_Machine();
        Run_Air_Filter(false);
        Run_Altitude_Filter(false);
        Run_Controllers();
        if (local_log_flag){
            Write_Output(false);
        }
    }
}

void Write_Output(bool initialize){
    // dont need to do this, just set data in e_Output_States
    e_Output_States.Position_NED[2] = -Altitude_Filter_data(0);
    e_Output_States.Velocity_NED[2] = -Altitude_Filter_data(1);
    e_Output_States.pressure = Barometer_Pressure();
    e_Output_States.Euler_deg[0] = Air_Filter_data(3)*R2D;
    e_Output_States.Euler_deg[1] = Air_Filter_data(4)*R2D;
    e_Output_States.Euler_deg[2] = Air_Filter_data(5)*R2D;
    e_Output_States.w_deg_s[0] = IMU_Angular_Rate(0) - Ground_Filter_data(3)*R2D;
    e_Output_States.w_deg_s[1] = IMU_Angular_Rate(1) - Ground_Filter_data(4)*R2D;
    e_Output_States.w_deg_s[2] = IMU_Angular_Rate(2) - Ground_Filter_data(5)*R2D;
    e_Output_States.v[0] = Air_Filter_data(0);
    e_Output_States.v[1] = Air_Filter_data(1);

}