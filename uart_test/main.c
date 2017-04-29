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
#include "pid.h"
#include "utils.h"
#include "encoders.h"

#define BAUDSEL 103 //9600baud




volatile bool overflowed = 0; 

void startUART() {
	UBRR1H = 0;
	UBRR1L = 25  ;
	
	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); //8 bits, no parity, one stop bit
	UCSR1B = _BV(TXEN1) | _BV(RXEN1); //enable TX and RX
}



void startTimer() {
	TCCR3B = _BV(CS31);
	TIMSK3 =  _BV(TOIE3);
	
	TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(CS01) | _BV(CS00);
}




ISR(TIMER3_OVF_vect) {
	overflowed = true;
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
		.Ki = 0.0005,
		.Kd = 0.5,
		.windupPrevention = MULTIPLY,
		.iTermConstraint = 0.95,
		.lastError = 0,
		.iTerm = 0
	};
	
	
	while (true) {
		if (overflowed) {
			overflowed = false;
			
			PORTC ^= _BV(6);
			
			int16_t desiredValue = 100;
			
			int16_t intVal = encoderB;
			
			while (!(UCSR1A & _BV(UDRE1)));
			UDR1 = intVal >> 8 ; 
			while (!(UCSR1A & _BV(UDRE1)));
			UDR1 = intVal & 0xFF; 
			
			int16_t error = 100 - encoderB;
			encoderB = 0;
						
			float correction = PID(&pidMotorB, error);
			
			float newValue = round((float)OCR0A + correction);
			
			OCR0A = truncateFloat(newValue, 0, 255);
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

