#ifndef GC_TYPES_H
#define GC_TYPES_H

#define DIAL_UPPER_BOUND 700
#define DIAL_LOWER_BOUND 350
#define DIAL_SENS 0.01f
#define DIAL_FIR_SIZE 30

typedef enum {
	// Drone systems initialized, awaiting calibration
	Standby,
	// Drone systems calibrating
	Calibrating,
	// Drone systems are calibrated, ready to fly
	Ready,
	// Drone is flying, responding to commands and under autopilot control
	Flying,
	// Drone is following landing procedure, will automatically proceed to ready once complete
	Landing
} FC_Status;

typedef enum {
	// Left-most dial
	Height_Dial,
	// Center dial
	North_Dial,
	// Right-most dial
	East_Dial
} Dial_ID;

// Structures
typedef struct {
	Dial_ID ID;
	unsigned char window_marker;
	unsigned int input_FIR[DIAL_FIR_SIZE];
	signed long window_left;
	signed long window_right;
	unsigned int reading;
	float step_size;
	float output;
} Dial;

typedef struct {
	unsigned char pin;
	char port;
} Led;

typedef struct {
	FC_Status Drone_status;
	unsigned char Pin;
	char Port;
} Button;

#endif