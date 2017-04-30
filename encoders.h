#ifndef ENCODERS_H
#define ENCODERS_H

#include <avr/io.h>
#include <avr/interrupt.h> 
#include <stdint.h>       
#include <stdbool.h>   

volatile uint8_t encoderState;

volatile int16_t encoderB;
volatile int16_t encoderA;

void startEncoding();
int16_t getTicksSinceLastA();
int16_t getTicksSinceLastB();



#endif