#ifndef MOTORS_H
#define MOTORS_H

#include <avr/io.h>
#include <stdint.h>

#include "pins.h"

void setMotorOut(uint8_t motor, uint8_t val);
void setupPWM();

#endif