#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>   // interrupt vectors
#include <util/delay.h>      // delay functions

#include <stdint.h>          // C header for int types like uint8_t
#include <stdbool.h>    
#include <stdio.h>      
#include <math.h>     

#include "pins.h"
#include "lcd.h"
#include "motors.h"
#include "sensors.h"

void setupPins();
int main();

#endif