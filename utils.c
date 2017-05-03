#include "utils.h"

float truncateFloat(float val, float lowLimit, float highLimit) {
	if (val > highLimit) {
		return highLimit;
	} else if (val < lowLimit) {
		return lowLimit;
	} else return val;
}

void pushBytes(void *dst, uint8_t ind, void *data, uint8_t len) {
	memcpy(dst + ind, data, len);
}