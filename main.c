#include "main.h"
// I was here

struct PIDConfig pidLine = {
	.Kp = 1.0,
	.Ki = 0,
	.Kd = 0,
	.windupPrevention = MULTIPLY,
	.iTermConstraint = 1,
	.lastError = 0,
	.iTerm = 0
};
struct PIDConfig pidMotorA = {
	.Kp = 0.015,
	.Ki = 0.0005,
	.Kd = 0.5,
	.windupPrevention = MULTIPLY,
	.iTermConstraint = 0.95,
	.lastError = 0,
	.iTerm = 0
};
struct PIDConfig pidMotorB = {
	.Kp = 0.015,
	.Ki = 0.0005,
	.Kd = 0.5,
	.windupPrevention = MULTIPLY,
	.iTermConstraint = 0.95,
	.lastError = 0,
	.iTerm = 0
};

enum STATE {
	STOP, FOLLOW, STRAIGHT
};

enum STATE robotState = STOP;
volatile bool nextTick = false; 
volatile uint32_t ticks = 0;
volatile uint32_t lastTicks = 0;

void setupTicks() {
	TCCR3B = _BV(CS31);
	TIMSK3 =  _BV(TOIE3);
}
ISR(TIMER3_OVF_vect) {
	ticks += 1;
}
int main() {
	setupPins(); //set data direction registers and pullups, absorb into below functions later
	setupADC(); //start reading line sensor values
	setupMotors(); //setup PWM timers
	setupEncoders(); //setup encoder reading interrupts and counter
	setupSerial(); //set serial baud rate and register buffers
	setupTicks(); //set up timing for pid loops
	uint8_t val = 0;
	sei();
	
	//see if initial calibration is better than continuous calibration?
	// while (!(PINC & _BV(7))) {
		// PORTD |= _BV(5);
	// }
	// PORTD &= ~_BV(5);
	// calibrating = false;
	
	int16_t avgSpeed = 100;
	uint16_t sendVal = 10;
	
	while (1) {
		if (ticks > lastTicks || ticks < lastTicks) {
			
			
			float lineError = getLineError();
			float correction = PID(&pidLine, lineError);
			correction = truncateFloat(correction, -1.0, 1.0);
			
			bool goLeft = correction < 0;
			float slowCorrection = 1 - fabs(correction);
			float fastCorrection = 1 + fabs(correction);
			int16_t slowSpeed = round(avgSpeed * slowCorrection);
			int16_t fastSpeed = round(avgSpeed * fastCorrection);
			
			
			int16_t speedB = goLeft ? slowSpeed : fastSpeed;
			int16_t speedA = !goLeft ? slowSpeed : fastSpeed;
			
			if (robotState == STRAIGHT) {
				int16_t speedB = avgSpeed;
				int16_t speedA = avgSpeed;
			}
			 
			int16_t ticksB = getEncoderDiffB() / (ticks - lastTicks);
			int16_t ticksA = getEncoderDiffA() / (ticks - lastTicks);
			int16_t errorB = speedB - ticksB;
			int16_t errorA = speedA - ticksA;
			float correctionB = PID(&pidMotorB, errorB);
			float correctionA = PID(&pidMotorA, errorA);
			float newValueB = round((float)getMotorOut(MB2) + correctionB);
			float newValueA = round((float)getMotorOut(MA2) + correctionA);
			
			if (robotState == STOP) {
				setMotorOut(MB2, 0);
				setMotorOut(MA2, 0);
			} else {
				setMotorOut(MB2, truncateFloat(newValueB, 0, 255));
				setMotorOut(MA2, truncateFloat(newValueA, 0, 255));
			}
			
			uint8_t outMsg[2] = {ticksB >> 8, ticksB & 0xFF};
			if (--sendVal == 0) {
				serialSend(outMsg, 2);
				sendVal = 10;
			}
			lastTicks = ticks;
		}
		uint8_t msgLen = serialRecv();
		if (msgLen >= 2) {
		}
		
		if (!(PIN_BTN & _BV(B_BTN0))) {
			robotState = (robotState + 1) % 3;
			_delay_ms(1000);
		} 
		
		switch (robotState) {
			case STOP:
				PORT_LED1 |= _BV(B_LED1);
				PORT_LED2 |= _BV(B_LED2);
				break;
			case FOLLOW:
				PORT_LED1 &= ~_BV(B_LED1);
				PORT_LED2 &= ~_BV(B_LED2);
				break;
			case STRAIGHT:
				PORT_LED1 &= ~_BV(B_LED1);
				PORT_LED2 |= _BV(B_LED2);
				break;
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
	
	//pullup resistors for  buttons
	PORT_BTN |= _BV(B_BTN0) | _BV(B_BTN1);
	
}