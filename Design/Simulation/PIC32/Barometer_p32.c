#include <stdint.h>
#include <string.h>
#include <math.h>
#include "Barometer_p32.h"

static BAR_Data barometer = {0};

static uint8_t offset_counter = 0;

void Initialize_Bar(){
    memset(&barometer, 0, sizeof(barometer));
    offset_counter = 0;
}

void Read_Bar(uint32_t pressure_LSB, uint8_t status){
    const double d_t = 1.0/200.0;
    const double IIR_c1 = 0.995;
    const double IIR_c2 = 1.0 - IIR_c1;

    const double sensitivity = 1.0/40.96; // Pa/LSB
    const double standard_temp = 288.15; // Standard temperature at sea level (K)
    const double gravity = 9.8065; // Acceleration due to gravity (m/s^2)
    const double lapse_rate = -0.0065; // Standard temperature lapse rate (K/m)
    const double standard_pressure = 101325.0; // Standard static pressure at sea level (Pa)
    const double gas_constant = 8.31432; // Universal gas constant (J/(mol-K))S
    const double molar_mass = 0.0289644; // Molar mass of air (kg/mol)
    
    const double c1 = standard_temp/lapse_rate;
    const double c2 = -(gas_constant*lapse_rate)/(gravity*molar_mass);

    barometer.pressure_LSB = pressure_LSB;
    barometer.pressure_pa = ((double)pressure_LSB)*sensitivity;
    barometer.height = c1*(pow(barometer.pressure_pa/standard_pressure, c2) - 1.0) - barometer.base_altitude;

    if (offset_counter == 0 && status){
        offset_counter++;
        barometer.base_altitude = barometer.height;
    }

    barometer.height_dot = barometer.height_dot_last*IIR_c1 + (IIR_c2*(barometer.height - barometer.height_last)/d_t);
    barometer.height_dot_last = barometer.height_dot;
    barometer.height_last = barometer.height;
}

double Barometer_Altitude(){
    return barometer.height;
}

double Barometer_Altitude_Dot(){
    return barometer.height_dot;
}

double Barometer_Pressure(){
    return barometer.pressure_pa;
}

// cpp wrapper functions

void Read_Bar_cpp(uint32_t pressure_LSB, uint8_t status){
    Read_Bar(pressure_LSB, status);
}

double Barometer_Altitude_cpp(){
    return Barometer_Altitude();
}

double Barometer_Pressure_cpp(){
    return Barometer_Pressure();
}

double Barometer_Altitude_Dot_cpp(){
    return Barometer_Altitude_Dot();
}

void Initialize_Bar_cpp(){
    Initialize_Bar();
}