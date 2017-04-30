#include "motors.h"

void setMotorOut(uint8_t motor, uint8_t val) {
	switch (motor) {
		case MA1:
		OCR1AL = val;
		break;
		case MA2:
		OCR1BL = val;
		break;
		case MB1:
		OCR0B = val;
		break;
		case MB2:
		OCR0A = val;
		break;
	}
}

void setupPWM() {
	OCR0A = 0; 
	TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(CS01) | _BV(CS00);
	//enable timer 0 with fast PWM with a 1/64 prescaler
	//results in a pwm frequency of about 976Hz
	//play around see what the motors like
	
	OCR1B = 0;
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
	TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10); 
	//clk/64 and only 8 bit fast PWM	
	//only 8 bits so we can use the same logic across both timers,
	//1024 bit precision is overkill with shit motors
}