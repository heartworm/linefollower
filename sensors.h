#ifndef SENSORS_H
#define SENSORS_H

#include <avr/io.h>
#include <avr/interrupt.h>   // interrupt vectors

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pins.h"

const uint16_t centerValue;
const uint8_t MUXES[];

volatile uint16_t readings[]; //10 bit output from the ADC
volatile uint32_t adjusted[]; //10 bit output from the ADC
volatile uint16_t maxes[]; //10 bit output from the ADC
volatile uint16_t mins[]; //10 bit output from the ADC
volatile uint8_t curMux;

bool calibrating;
uint16_t lastPos;
bool lostLine;

void setupADC();
void setMux(int mux);
uint16_t getCoL();
bool isLineLost();

#endif