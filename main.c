#include "main.h"
// I was here

float iTerm = 0;
float lastError = 0;

int main() {
	setupPins();
	setupADC(); //start reading line sensor values
	setupPWM(); //setup PWM timers
	uint8_t val = 0;
	calibrating = true; 
	DDRD &= ~_BV(4); //jumper for straight ahead, set as input
	PORTD |= _BV(4); //pullup resistor
	
	//see if initial calibration is better than continuous calibration?
	// while (!(PINC & _BV(7))) {
		// PORTD |= _BV(5);
	// }
	// PORTD &= ~_BV(5);
	// calibrating = false;
	
	
	while (1) {
		
	
		if (PINC & _BV(6)) {
			val = val >= 127 ? 127 : val + 1;
			//fast wheel can go up to double speed at full lock steering, so max is 255/2 ~= 127
		} 
		if (PINC & _BV(7)) {
			val = val <= 0 ? 0 : val - 1;
		}
		_delay_ms(10);
		
		//show the estimated line position on Top LEDs
		//if we lose the line, turn on the blue LED. 
		uint16_t pos = getCoL();
		if (pos < 1000) {
		} else if (pos < 1500) {
		} else if (pos < 2500) {
		} else if (pos < 3000) {
		} else {	
		}
		
		const float Kp = 1.5;
		const float Ki = 0.05;
		const float Kd = 0.25;
		
		float error = (pos - 2000.0) / 2000.0;
		
		iTerm += error; 
		//stop integral windup if the bot gets stuck etc
		if (fabs(iTerm) > 1) { //turn this limiting into a function
			iTerm = iTerm < 0 ? -1 : 1;
		}
		
		float dTerm = lastError - error;
		lastError = error;
		
		float correction = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);
		if (fabs(correction) > 1) {
			correction = correction < 0 ? -1 : 1;
		}	
		
		// bool goLeft = correction < 0;
		// correction = 1 - fabs(correction);
		// uint16_t slowMotorVal = round(val * correction);
		// slowMotorVal = slowMotorVal > val ? val : slowMotorVal;
		
		bool goLeft = correction < 0;
		float slowCorrection = 1 - fabs(correction);
		float fastCorrection = 1 + fabs(correction);
		uint16_t slowMotorVal = round(val * slowCorrection);
		uint16_t fastMotorVal = round(val * fastCorrection);
		
		slowMotorVal = slowMotorVal > 255 ? 255 : slowMotorVal;
		fastMotorVal = fastMotorVal > 255 ? 255 : fastMotorVal;
		
		
		uint8_t leftMotorVal = goLeft ? slowMotorVal : fastMotorVal;
		uint8_t rightMotorVal = !goLeft ? slowMotorVal : fastMotorVal;
		
		
		
		if (PIND & _BV(4)) { //if jumper not connected
			setMotorOut(MB1, leftMotorVal);
			setMotorOut(MA1, rightMotorVal);
		} else {			
			setMotorOut(MB1, val);
			setMotorOut(MA1, val);
		}
		
	}
	
	return 1;
}

void setupPins() {
	//outputs for motor driver
	DDRB |= _BV(B_MA1) | _BV(B_MA2) | _BV(B_MB2); 
	DDRD |= _BV(B_MB1);
	//outputs for board LEDs
    DDRD |= _BV(B_LED1);
    DDRC |= _BV(B_LED2) | _BV(B_LED0);
	//output for onboard LED
	DDRD |= _BV(5);
	
}