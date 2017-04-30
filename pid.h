#ifndef PID_H
#define PID_H

#include "utils.h"

enum WUP_TYPE {
	MULTIPLY, TRUNCATE, NONE
};

struct PIDConfig {
	float Kp;
	float Ki;
	float Kd;
		
	enum WUP_TYPE windupPrevention;
	float iTermConstraint;
	
	float lastError;
	float iTerm;
};

float PID(struct PIDConfig *config, float error);

#endif