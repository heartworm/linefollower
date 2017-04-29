#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h> 
#include <util/delay.h>    
#include <stdint.h>       
#include <stdbool.h>    
#include <stdio.h>      
#include <stdlib.h>
#include <math.h>     

#include "pins.h"
// #include "uart.h"

#define BAUDSEL 103 //9600baud

enum WUP_TYPE {
	MULTIPLY, TRUNCATE, NONE
}

struct PIDConfig {
	float Kp;
	float Ki;
	float Kd;
		
	enum WUP_TYPE windupPrevention;
	float iTermConstraint;
	
	volatile float lastError;
	volatile float iTerm;
}

volatile uint8_t encoderState = 0;

volatile int16_t encoderB = 0;
volatile uint16_t encoderA = 0;

volatile bool overflowed = 0; 


volatile float iTermEncoderB = 0.0;
volatile int16_t prevErrorB = 0.0;

void startUART() {
	UBRR1H = 0;
	UBRR1L = 25  ;
	
	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); //8 bits, no parity, one stop bit
	UCSR1B = _BV(TXEN1) | _BV(RXEN1); //enable TX and RX
}

void startEncoding() {
	PCMSK0 = _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3);
	PCICR = _BV(PCIE0);
}

void startTimer() {
	TCCR3B = _BV(CS31);
	TIMSK3 =  _BV(TOIE3);
	
	TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(CS01) | _BV(CS00);
}


ISR(PCINT0_vect) {
	//XOR outputs a 1 if the values are different, so shows us which bits have changed
	uint8_t changeMask = encoderState ^ PINB;
	//Compare Encoder A to the previous B value, if they're different/not different
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

ISR(TIMER3_OVF_vect) {
	overflowed = true;
}

float truncateFloat(float val, float limit) {
	if (fabs(val) > limit) {
		return val > 0 ? limit : limit * -1;
	} else return val;
}


float PID(struct PIDConfig *config, float error) {
	float dTerm = error - config->lastError;
	config->lastError = error;
	
	switch (config->windupPrevention) {
		case MULTIPLY:
			config->iTerm = (config->iTerm * config->iTermConstraint) + error;
			break;
		case TRUNCATE:
			config->iTerm += error;
			config->iTerm = truncateFloat(config->iTerm, config->iTermConstraint);
			break;
		case NONE:
			config->iTerm += error;
			break;
	}
	
	float correction = (config->Kp * error) + (config->Ki * config->iTerm) + (config->Kd * dTerm);
	return correction;
}

int main() {
	PORT_BTN |= _BV(B_BTN0) | _BV(B_BTN1);
	
	DDRD |= _BV(B_MB1);
	DDRB |= _BV(B_MB2);
	DDRC |= _BV(6);
	
	PORTC |= _BV(6);
	_delay_ms(1000);
	PORTC &= ~_BV(6);
	
	startUART();
	startEncoding();
	startTimer();
	sei();
	
	struct PIDConfig pidMotorB = {
		.Kp = 0.01,
		.Ki = 0.002,
		.Kd = 0,
		.windupPrevention = MULTIPLY,
		.iTermConstraint = 0.9,
		.lastError = 0,
		.iTerm = 0
	}
	
	
	while (true) {
		if (overflowed) {
			overflowed = false;
			
			PORTC ^= _BV(6);
			
			int16_t intVal = encoderB;
			
			while (!(UCSR1A & _BV(UDRE1)));
			UDR1 = intVal >> 8 ; 
			while (!(UCSR1A & _BV(UDRE1)));
			UDR1 = intVal & 0xFF; 
			
			int16_t error = desiredValue - encoderB;
			encoderB = 0;
			
			float dTermEncoderB = error - prevErrorB;
			prevErrorB = error;
			
			iTermEncoderB = (iTermEncoderB * 0.9) + error;

			float Kp = 0.01;
			float Ki = 0.002;
			float Kd = 0;
			
			float correction = (error * Kp) + (iTermEncoderB * Ki) + (dTermEncoderB * Kd);
			
			
			float newValue = round((float)OCR0A + correction);
			newValue = newValue > 255 ? 255 : newValue < 0 ? 0 : newValue;
			

			
			OCR0A = newValue;
		}
		

		// if (!(PIN_BTN & _BV(B_BTN0))) {
			// OCR0A = OCR0A >= 255 ? OCR0A : OCR0A + 1;
			// _delay_ms(50);
		// } else if (!(PIN_BTN & _BV(B_BTN1))) {
			// OCR0A = OCR0A <= 0 ? OCR0A : OCR0A - 1;
			// _delay_ms(50);
		// }
		
		
	}
	
	
	return 0;
	
}

