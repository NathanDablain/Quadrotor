#ifndef MOTORS_H
#define	MOTORS_H

#include <stdint.h>

#define MIN_DUTY_CYCLE 1000
#define MAX_DUTY_CYCLE 2000
#define MIN_THROTTLE   0
#define MAX_THROTTLE   1000

void Apply_Throttle(uint16_t throttle, uint8_t index);

void Apply_Throttle_Batch(uint16_t throttle[4]);

void Calibrate_Motors();

#endif

