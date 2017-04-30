#ifndef MOTORS_H
#define MOTORS_H

#include <avr/io.h>
#include <stdint.h>

#include "pins.h"

enum MOTOR {MA1, MA2, MB1, MB2};

void setupMotors();
void setMotorOut(enum MOTOR motor, uint8_t val);
uint8_t getMotorOut(enum MOTOR motor);

#endif