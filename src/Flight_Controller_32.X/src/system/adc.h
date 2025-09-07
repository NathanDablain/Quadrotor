#ifndef ADC_H
#define	ADC_H

#include <xc.h>
#include <stdint.h>

#define ADC_1S_MUX 9
#define ADC_4S_MUX 0
#define ADC_15_16_AVCC_MUX 14

void Initialize_ADC();

void Trigger_ADC();

#endif

