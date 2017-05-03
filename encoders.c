#include "encoders.h"

volatile uint8_t encoderState = 0;
volatile int16_t encoderB = 0;
volatile int16_t encoderA = 0;


ISR(PCINT0_vect) {
	//XOR outputs a 1 if the values are different, so shows us which bits (pins) have changed
	uint8_t changeMask = encoderState ^ PINB;
	//Compare one encoder pin to the previous value of the other pin, if they're different/not different
	//The encoder is spinning one of two ways. 
	uint8_t dirMask = encoderState ^ (PINB >> 1);
	
	//look at the change mask, which motor reported the change?
	bool changeB = changeMask & (_BV(0) | _BV(1));
	bool changeA = changeMask & (_BV(2) | _BV(3));
	
	//If both values changed in the encoder, we've lost an intermediary step. 
	bool invalidB = (changeMask & _BV(0)) & (changeMask >> 1);
	bool invalidA = (changeMask & _BV(2)) & (changeMask >> 1);
	
	
	if (changeB && !invalidB) {
		//depending on the  direction, increment or decrement the wheel encoder count
		encoderB += dirMask & _BV(0) ? 1 : -1; 
	}
	if (changeA && !invalidA) {
		encoderA += dirMask & _BV(2) ? 1 : -1; 
	}
		
		encoderState = PINB;
}

void setupEncoders() {
	PCMSK0 = _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3);
	PCICR = _BV(PCIE0);
}

int16_t getEncoderDiffA() {
	int16_t val = encoderA;
	encoderA = 0;
	return val;
}

int16_t getEncoderDiffB() {
	int16_t val = encoderB;
	encoderB = 0;
	return val;
}