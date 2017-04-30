#include "main.h"
// I was here

struct PIDConfig pidLine = {
	.Kp = 1.5,
	.Ki = 0.05,
	.Kd = 0.25,
	.windupPrevention = TRUNCATE,
	.iTermConstraint = 1,
	.lastError = 0,
	.iTerm = 0
};
struct PIDConfig pidMotorA = {
	.Kp = 0.01,
	.Ki = 0.0005,
	.Kd = 0.5,
	.windupPrevention = MULTIPLY,
	.iTermConstraint = 0.95,
	.lastError = 0,
	.iTerm = 0
};
struct PIDConfig pidMotorB = {
	.Kp = 0.01,
	.Ki = 0.0005,
	.Kd = 0.5,
	.windupPrevention = MULTIPLY,
	.iTermConstraint = 0.95,
	.lastError = 0,
	.iTerm = 0
};

volatile bool nextTick = false; 

void setupTicks() {
	TCCR3B = _BV(CS31) | _BV(CS30);
	TIMSK3 =  _BV(TOIE3);
}
ISR(TIMER3_OVF_vect) {
	nextTick = true;
}
bool isNextTick() {
	bool output = nextTick;
	nextTick = false;
	return output;
}

int main() {
	setupPins(); //set data direction registers and pullups, absorb into below functions later
	setupADC(); //start reading line sensor values
	setupMotors(); //setup PWM timers
	setupEncoders(); //setup encoder reading interrupts and counter
	setupSerial(); //set serial baud rate and register buffers
	setupTicks(); //set up timing for pid loops
	uint8_t val = 0;
	
	//see if initial calibration is better than continuous calibration?
	// while (!(PINC & _BV(7))) {
		// PORTD |= _BV(5);
	// }
	// PORTD &= ~_BV(5);
	// calibrating = false;
	
	int16_t desiredValue = 800;
	uint16_t sendVal = 5;
	
	while (1) {
		if (isNextTick()) {
			// float correction = PID(&pidLine, getLineError());
			// correction = truncateFloat(correction, -1.0, 1.0);
			
			// bool goLeft = correction < 0;
			// float slowCorrection = 1 - fabs(correction);
			// float fastCorrection = 1 + fabs(correction);
			// uint16_t slowMotorVal = round(val * slowCorrection);
			// uint16_t fastMotorVal = round(val * fastCorrection);
			
			// slowMotorVal = slowMotorVal > 255 ? 255 : slowMotorVal;
			// fastMotorVal = fastMotorVal > 255 ? 255 : fastMotorVal;
			
			// uint8_t leftMotorVal = goLeft ? slowMotorVal : fastMotorVal;
			// uint8_t rightMotorVal = !goLeft ? slowMotorVal : fastMotorVal;
			
			// setMotorOut(MB1, leftMotorVal);
			// setMotorOut(MA1, rightMotorVal);
			
			
			int16_t ticksB = getEncoderDiffB();
			int16_t error = desiredValue - ticksB;
			float correction = PID(&pidMotorB, error);
			float newValue = round((float)getMotorOut(MB2) + correction);
			setMotorOut(MB2, truncateFloat(newValue, 0, 255));
			//setMotorOut(MB2, 100);
			
			uint8_t outMsg[2] = {ticksB >> 8, ticksB & 0xFF};
			//if (--sendVal == 0) {
				serialSend(outMsg, 2);
				sendVal = 5;
			//}
			
		}
		uint8_t msgLen = serialRecv();
		if (msgLen >= 2) {
			uint8_t *msg = serialGetMsgBuffer();
			desiredValue = (msg[0] << 8) | (msg[1]);
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