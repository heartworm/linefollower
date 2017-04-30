#include "pid.h"

float PID(struct PIDConfig *config, float error) {
	float dTerm = error - config->lastError;
	config->lastError = error;
	
	switch (config->windupPrevention) {
		case MULTIPLY:
			config->iTerm = (config->iTerm * config->iTermConstraint) + error;
			break;
		case TRUNCATE:
			config->iTerm += error;
			config->iTerm = truncateFloat(config->iTerm, config->iTermConstraint * -1.0, config->iTermConstraint);
			break;
		case NONE:
			config->iTerm += error;
			break;
	}
	
	float correction = (config->Kp * error) + (config->Ki * config->iTerm) + (config->Kd * dTerm);
	return correction;
}