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
	.Kp = 0.02,
	.Ki = 0.0005,
	.Kd = 0.5,
	.windupPrevention = MULTIPLY,
	.iTermConstraint = 0.95,
	.lastError = 0,
	.iTerm = 0
};
struct PIDConfig pidMotorB = {
	.Kp = 0.03,
	.Ki = 0.0015,
	.Kd = 0.5,
	.windupPrevention = MULTIPLY,
	.iTermConstraint = 0.95,
	.lastError = 0,
	.iTerm = 0
};

struct {
	uint16_t centerOfLine;
	int16_t avgSpeed;
	int16_t valA;
	int16_t speedA;
	int16_t actualA;
	int16_t valB;
	int16_t speedB;
	int16_t actualB;
} coreState = {0,100,0,0,0,0,0,0};


enum MODE {
	STOP, FOLLOW, STRAIGHT
};

enum MODE robotMode = STRAIGHT;
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
	
	int16_t avgSpeed = -100;
	uint16_t sendVal = 10;
	
	while (1) {
		if (ticks > lastTicks || ticks < lastTicks) {
			
			coreState.centerOfLine = getCoL();
			float lineError = getLineError();
			float correction = PID(&pidLine, lineError);
			correction = truncateFloat(correction, -1.0, 1.0);
			
			bool goLeft = correction < 0;
			float slowCorrection = 1 - fabs(correction);
			float fastCorrection = 1 + fabs(correction);
			int16_t slowSpeed = round(coreState.avgSpeed * slowCorrection);
			int16_t fastSpeed = round(coreState.avgSpeed * fastCorrection);
			
			if (robotMode == STRAIGHT) {
				coreState.speedB = coreState.avgSpeed;
				coreState.speedA = coreState.avgSpeed;
			} else {
				coreState.speedB = goLeft ? slowSpeed : fastSpeed;
				coreState.speedA = !goLeft ? slowSpeed : fastSpeed;
			}
			 
			coreState.actualB = getEncoderDiffB() / (ticks - lastTicks);
			coreState.actualA = getEncoderDiffA() / (ticks - lastTicks);
			int16_t errorB = coreState.speedB - coreState.actualB;
			int16_t errorA = coreState.speedA - coreState.actualA;
			float correctionB = PID(&pidMotorB, errorB);
			float correctionA = PID(&pidMotorA, errorA);
			
			//if we want to move the wheel one way but we power it another way, set it to 0 before applying the correction
			// if ((coreState.valB * coreState.speedB) < 0) {
				// coreState.valB = 0;
			// }
 			// if ((coreState.valA * coreState.speedA) < 0) { //differing signs
				// coreState.valA = 0;
			// } 
			
			coreState.valB = coreState.speedB == 0 ? 0 : round(truncateFloat((float)coreState.valB + correctionB, -255, 255));
			coreState.valA = coreState.speedA == 0 ? 0 : round(truncateFloat((float)coreState.valA + correctionA, -255, 255));
			
			if (robotMode == STOP) {
				setMotorOut(MB2, 0);
				setMotorOut(MB1, 0);
				setMotorOut(MA1, 0);
				setMotorOut(MA2, 0);
			} else {
				if (coreState.valB >= 0) {
					setMotorOut(MB1, 0);
					setMotorOut(MB2, coreState.valB);
				} else {
					setMotorOut(MB2, 0);
					setMotorOut(MB1, abs(coreState.valB));
				}
				
				if (coreState.valA >= 0)	{
					setMotorOut(MA1, 0);
					setMotorOut(MA2, coreState.valA);
				} else {
					setMotorOut(MA2, 0);
					setMotorOut(MA1, abs(coreState.valA));
				}
			}
			
			// uint8_t outMsg[2] = {ticksB >> 8, ticksB & 0xFF};
			// if (--sendVal == 0) {
				// serialSendEscaped(outMsg, 2);
				// sendVal = 10;
			// }
			lastTicks = ticks;
		}
		
		uint8_t msgLen = serialRecv();
		uint8_t *msg = serialGetMsgBuffer();
		if (msgLen >= 1) {
			uint8_t hdr = msg[0];
			if (hdr == 0x01 && msgLen == 3) {
				coreState.avgSpeed = (msg[1] << 8) | msg[2];
			}
			
			if (hdr == 0x00 && msgLen == 1) {
				uint8_t outMsg[17];
				outMsg[0] = 0x00;
				pushBytes(outMsg, 1, &coreState.centerOfLine, 2);
				pushBytes(outMsg, 3, &coreState.avgSpeed, 2);
				pushBytes(outMsg, 5, &coreState.valA, 2);
				pushBytes(outMsg, 7, &coreState.speedA, 2);
				pushBytes(outMsg, 9, &coreState.actualA, 2);
				pushBytes(outMsg, 11, &coreState.valB, 2);
				pushBytes(outMsg, 13, &coreState.speedB, 2);
				pushBytes(outMsg, 15, &coreState.actualB, 2);
				serialSendEscaped(outMsg, 17);
			}
			
		}
		
		if (!(PIN_BTN & _BV(B_BTN0))) {
			robotMode = (robotMode + 1) % 3;
			_delay_ms(1000);
		} 
		
		switch (robotMode) {
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