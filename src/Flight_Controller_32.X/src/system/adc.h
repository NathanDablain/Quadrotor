#ifndef ADC_H
#define	ADC_H

#include <xc.h>
#include <stdint.h>

#define ADC_1S_MUX 9
#define ADC_4S_MUX 0
#define ADC_15_16_AVCC_MUX 14

#define ADC_1S_CHANNEL 0
#define ADC_4S_CHANNEL 1

void Initialize_ADC();

void Trigger_ADC(uint8_t channel);

#endif

