#ifndef PINS_H
#define	PINS_H

#include <xc.h>
#include <stdint.h>

#define RED_LED_PIN 2
#define RED_LED_PORT LATB
#define YELLOW_LED_PIN 9
#define YELLOW_LED_PORT LATB
#define GREEN_LED_PIN 8
#define GREEN_LED_PORT LATB

#define ADC_1S_PIN 9
#define ADC_1S_PORT LATA
#define ADC_4S_PIN 2
#define ADC_4S_PORT LATA

#define CS_MAG_PORT LATC
#define CS_MAG_PIN 8
#define CS_BAR_PORT LATB
#define CS_BAR_PIN 11
#define CS_IMU_PORT LATB
#define CS_IMU_PIN 10

#define CS_LORA_PORT LATD
#define CS_LORA_PIN 7
#define LORA_BUSY_PORT LATD
#define LORA_BUSY_PIN 6

#define CS_AVR_PORT LATB
#define CS_AVR_PIN 1

#define CS_ARDUINO_PORT LATB
#define CS_ARDUINO_PIN 1

#define RAISE_PIN(port, pin) (port |= (1<<pin))
#define LOWER_PIN(port, pin) (port &= ~(1<<pin))

void Setup_Pins();

#endif

