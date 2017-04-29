#include "utils.h"

float truncateFloat(float val, float lowLimit, float highLimit) {
	if (val > highLimit) {
		return highLimit;
	} else if (val < lowLimit) {
		return lowLimit;
	} else return val;
}