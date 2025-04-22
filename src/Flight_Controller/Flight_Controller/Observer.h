#ifndef OBSERVER_H
#define OBSERVER_H

#include "FC_Types.h"

// Increasing the gain increases the trust on the model(gyro), and decreasing it increases the trust on the measurement (accel & mag)
#define OBSERVER_GAIN 0.05f
// Time step between observer predictions
#define OBSERVER_DT 0.005f
// Angle at which to stop making predictions and rely entirely on measurements because of pitch discontinuity
#define OBSERVER_GIMBAL_LOCK_CHECK (5.0f*D2R)

void Observer(States *Drone);

void Observer_Update(States *Drone);

void Observer_Predict(States *Drone);

#endif