#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <string.h>

float truncateFloat(float val, float lowLimit, float highLimit);
void pushBytes(void *dst, uint8_t ind, void *data, uint8_t len);

#endif