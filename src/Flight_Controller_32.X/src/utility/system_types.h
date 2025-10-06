#ifndef SYSTEM_TYPES_H
#define	SYSTEM_TYPES_H

typedef enum {
	// Drone systems initialized, awaiting calibration
	Standby,
	// Drone systems are being calibrated by the user, involves rotating drone body
	User_Calibration,
	// Drone systems are performing automatic pre flight calibration, drone should be in position
    System_Calibration,
	// Drone systems are calibrated, ready to fly
	Ready,
	// Drone is flying, responding to commands and under autopilot control
	Flying,
	// Drone is following landing procedure, will automatically proceed to ready once complete
	Landing
} FC_Status;

#endif

