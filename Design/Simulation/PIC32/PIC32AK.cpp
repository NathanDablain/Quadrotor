#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
#include <assert.h>
#include <iomanip>
#include "PIC32AK.h"
#include "Kalman_Filter_p32.h"
#include "Linear_Algebra_p32.h"

PIC32AK::PIC32AK(){
    Flight_Controller_Status = Standby_p32;
    Air_Filter_Initialized = false;
    Initialize_Guidance_Machine_cpp();
    Initialize_Controllers_cpp();

    Gyro_Update_Time.tmr1_count = 25*Gyro_Rate_us;
    Accel_Update_Time.tmr1_count = 25*Accel_Rate_us;
    Mag_Update_Time.tmr1_count = 25*Mag_Rate_us;
    Bar_Update_Time.tmr1_count = 25*Bar_Rate_us;
    Print_Update_Time.tmr1_count = 25*Print_Rate_us;
    Thrust_Controller_Update_Time.tmr1_count = 25*Thrust_Controller_Rate_us;
    Moment_Controller_Update_Time.tmr1_count = 25*Moment_Controller_Rate_us;
    Air_Filter_Update_Time.tmr1_count = 25*Air_Filter_Rate_us;

    Set_ODR_cpp(3330.0, 1660.0);
    IMU_Filter_Status = Initialize_IMU_Filters_cpp();
    Initialize_Bar_cpp();
    Initialize_Mag_cpp();
}

void PIC32AK::Run(Environment &env, Sim_Time sim_t){
    Time Current_Time = {.seconds = sim_t.Seconds, .tmr1_count = (25*sim_t.MicroSeconds)};

    // Sample sensors
    if (Flight_Controller_Status >= Flying_p32 && !Air_Filter_Initialized){
        Initialize_Air_Filter_cpp();
        Air_Filter_Initialized = true;
    }

    if (Compare_And_Update_cpp(Current_Time, Gyro_Update_Time, &Gyro_Timelast)){
        Read_Gyro_cpp(Current_Time, &imu.angular_rate_LSB[0]);
        if (IMU_Filter_Status){
            if (Flight_Controller_Status == System_Calibration_p32){
                Ground_Filter_Predict_cpp();
            }
            else if (Flight_Controller_Status >= Flying_p32){
                imu.Set_Filter_Settings(4, 3);
                Air_Filter_Predict_cpp();
            }
        }
    }

    if (Compare_And_Update_cpp(Current_Time, Accel_Update_Time, &Accel_Timelast)){
        Read_Accel_cpp(&imu.acceleration_LSB[0]);
        if (Flight_Controller_Status >= Flying_p32){
            Altitude_Filter_Predict_cpp();
        }
    }

    if (Compare_And_Update_cpp(Current_Time, Mag_Update_Time, &Mag_Timelast)){
        Read_Mag_cpp(&magnetometer.magnetic_field_LSB[0]);
        double mag_field[3] = {Magnetometer_Field_cpp(0), Magnetometer_Field_cpp(1), Magnetometer_Field_cpp(2)};
        if (IMU_Filter_Status){
            if (Flight_Controller_Status == System_Calibration_p32){
                Ground_Filter_Update_cpp(mag_field);
            }
            // else if (Flight_Controller_Status >= Flying_p32 && Barometer_Altitude_cpp() > 0.5){
            //     Air_Filter_Update_cpp();
            // }
        }
    }

    if (Compare_And_Update_cpp(Current_Time, Air_Filter_Update_Time, &Air_Filter_Timelast)){
        if (Flight_Controller_Status >= Flying_p32 && Barometer_Altitude_cpp() > 0.5){
            Air_Filter_Update_cpp();
        }
    }

    if (Compare_And_Update_cpp(Current_Time, Bar_Update_Time, &Bar_Timelast)){
        uint8_t bar_status = (Flight_Controller_Status>=System_Calibration_p32)?(1):(0);
        Read_Bar_cpp(barometer.Pressure_Out_LSB, bar_status);
        if (Flight_Controller_Status >= Flying_p32){
            Altitude_Filter_Update_cpp();
        }
    }
    
    if (Flight_Controller_Status >= Flying_p32){
        Guidance_Machine_cpp(Current_Time.seconds);

        if (Compare_And_Update_cpp(Current_Time, Thrust_Controller_Update_Time, &Thrust_Controller_Timelast)){
            Thrust_Control_cpp();
        }

        if (Compare_And_Update_cpp(Current_Time, Moment_Controller_Update_Time, &Moment_Controller_Timelast)){
            Moment_Control_cpp();
        }

        Read_Throttles_cpp(mapped_throttle_commands);
    }

    // Check Guidance State
    if (Flight_Controller_Status == Flying_p32 && Get_Guidance_State_cpp() == Landed){
        Flight_Controller_Status = Standby_p32;
        Successful_Landing = true;
    }

    // Write data to output structure
    Prep_Output();
    
    // Print Output for debugging
    if (print_flag){
        if (Compare_And_Update_cpp(Current_Time, Print_Update_Time, &Print_Timelast)){
            std::cout.flush();
            std::cout << setprecision(6) << setw(8) << static_cast<double>(Current_Time.seconds) << setw(3) << "  ";
            switch (Flight_Controller_Status){
                case Standby_p32:
                    break;

                case User_Calibration_p32:
                    break;

                case System_Calibration_p32:
                    for (uint8_t i = 0; i < 9; i++){
                        std::cout << setw(8) << Filter_data_cpp(i, 0) << setw(3) << "  ";
                    }
                    break;
                case Flying_p32:
                    for (uint8_t i = 0; i < 4; i++){
                        std::cout << setw(8) << mapped_throttle_commands[i] << setw(3) << "  ";
                    }
                    break;

                default:
                    break;
            }
            std::cout << std::endl;
        }
    }

}

void PIC32AK::Prep_Output(){
    Output_States.w_deg_s[0] = IMU_Angular_Rate_cpp(0) - (Filter_data_cpp(3, 0)*R2D);
    Output_States.w_deg_s[1] = IMU_Angular_Rate_cpp(1) - (Filter_data_cpp(4, 0)*R2D);
    Output_States.w_deg_s[2] = IMU_Angular_Rate_cpp(2) - (Filter_data_cpp(5, 0)*R2D);
    Output_States.Euler_deg[0] = Filter_data_cpp(3, 1)*R2D;
    Output_States.Euler_deg[1] = Filter_data_cpp(4, 1)*R2D;
    Output_States.Euler_deg[2] = Filter_data_cpp(5, 1)*R2D;
    Output_States.v[0] = Filter_data_cpp(0, 1);
    Output_States.v[1] = Filter_data_cpp(1, 1);
    Output_States.Position_NED[2] = -Barometer_Altitude_cpp();
    Output_States.pressure = Barometer_Pressure_cpp();
    Output_States.Velocity_NED[2] = -Barometer_Altitude_Dot_cpp();
    Output_States.Altitude_Filter_Data[0] = Filter_data_cpp(0, 2);
    Output_States.Altitude_Filter_Data[1] = Filter_data_cpp(1, 2);
}
