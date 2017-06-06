#include "main.h"
// I was here

struct PIDConfig pidLine = {
	.Kp = 1.75,
	.Ki = 0.05,
	.Kd = .75,
	.windupPrevention = TRUNCATE,
	.iTermConstraint = 1,
	.lastError = 0,
	.iTerm = 0
};
struct PIDConfig pidMotorA = {
	.Kp = 0.03,
	.Ki = 0.0015,
	.Kd = 0.75,
	.windupPrevention = MULTIPLY,
	.iTermConstraint = 0.95,
	.lastError = 0,
	.iTerm = 0
};
struct PIDConfig pidMotorB = {
	.Kp = 0.03,
	.Ki = 0.0015,
	.Kd = 0.75,
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
} coreState = {0,400,0,0,0,0,0,0};


enum MODE {
	STOP, FOLLOW, STRAIGHT
};

enum MODE robotMode = STOP;
volatile bool nextTick = false; 
volatile uint32_t ticks = 0;
volatile uint32_t lastTicks = 0;

float movingAvg = 0;
uint16_t cornerCheck = 0;
uint16_t wasLeftMarker = 0;
bool wasRightMarker = false;

uint16_t curvatureCount = 0;

bool wasCorner = false;
bool isSlowZone = false;
uint16_t slowZoneCount = 0;
uint16_t slowZoneTicks = 100;

bool startStop = true;
uint16_t startStopCount = 0;

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
	sei();
	
	
	while (1) {
		if (ticks > lastTicks || ticks < lastTicks) {		
			
			coreState.centerOfLine = getCoL();
			float lineError = toLineError(coreState.centerOfLine);
			
			float correction = PID(&pidLine, lineError);
			correction = truncateFloat(correction, -1.0, 1.0);
			
			movingAvg = (correction * 0.1) + (correction * (1-0.1));
			
			float absAvg = fabs(movingAvg);
			bool isCorner = absAvg >= 0.15;
			
			
			if (wasCorner != isCorner) {
				curvatureCount = 0;
			} else {
				curvatureCount = curvatureCount + 1 < curvatureCount ? curvatureCount : curvatureCount + 1;
			}
			
			float slowAmt = 200.0 * truncateFloat(absAvg / 0.15, 0, 1);			
			if (absAvg <= 0.05) slowAmt = 0;
			coreState.avgSpeed = 400.0 - slowAmt;
			
			if (isSlowZone || (curvatureCount > 3 && isCorner)) {
				// coreState.avgSpeed = 150;
				PORT_BZR &= ~_BV(B_BZR);
			} else if (curvatureCount > 10 && !isCorner) {
				// coreState.avgSpeed = 300;
				PORT_BZR |= _BV(B_BZR);
			}
			
			
			
			wasCorner = isCorner;
			
			// if (isCorner) {
				// coreState.avgSpeed = 100;
			// } else {
				// coreState.avgSpeed = 250;
			// }
			
			bool goLeft = correction < 0;
			float slowCorrection = 1 - fabs(correction);
			float fastCorrection = 1 + fabs(correction);
			int16_t slowSpeed = round(coreState.avgSpeed * slowCorrection);
			int16_t fastSpeed = round(coreState.avgSpeed * fastCorrection);
			
			if (robotMode == STRAIGHT) {
				coreState.speedB = coreState.avgSpeed;
				coreState.speedA = coreState.avgSpeed;
			} else {
				coreState.speedA = goLeft ? slowSpeed : fastSpeed;
				coreState.speedB = !goLeft ? slowSpeed : fastSpeed;
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
				if (coreState.valB >= 0) { //forwards
					setMotorOut(MB2, 0);
					setMotorOut(MB1, coreState.valB);
				} else { //backwards
					setMotorOut(MB1, 0);
					setMotorOut(MB2, abs(coreState.valB));
				}
				
				if (coreState.valA >= 0)	{ //forwards
					setMotorOut(MA2, 0);
					setMotorOut(MA1, coreState.valA);
				} else { //backwards
					setMotorOut(MA1, 0);
					setMotorOut(MA2, abs(coreState.valA));
				}
			}
			
			//Corner detection
			bool nowOnLeftMarker = getLeftCornerVal() > 400;
			bool nowOnRightMarker = readings[IND_RC] > 900;
			
			if (!nowOnRightMarker && wasRightMarker && !wasLeftMarker) {
				if (startStop) startStop = false;
				else {
					startStop = true;
					robotMode = STOP;
					startStopCount = 100;
				}
			}
			
			// if (isSlowZone && !wasLeftMarker && nowOnLeftMarker) {
				// isSlowZone = false;
				// slowZoneTicks = 0;
				// PORTD &= ~_BV(5);
			// }
			// if (slowZoneTicks < 100) {
				// slowZoneTicks++;
			// }
			
			
			
			if (startStopCount > 0) {
				if (--startStopCount == 0) robotMode = FOLLOW;
			}
			
			if (nowOnLeftMarker) {
				wasLeftMarker = 5;
			} else if (wasLeftMarker != 0) {
				wasLeftMarker--;
			}
			wasRightMarker = nowOnRightMarker;
			
			lastTicks = ticks;
			
			
		}
		
		// if (readings[IND_LC] >= 800 && readings[IND_LC] <= 850){
			// slowZoneCount++;
		// } else {
			// slowZoneCount = 0;
		// }
		
		// if (slowZoneCount >= 200 /* && slowZoneTicks >= 100 */) {
			// isSlowZone = true;
			// PORTD |= _BV(5);
		// }
		
		uint8_t msgLen = serialRecv();
		uint8_t *msg = serialGetMsgBuffer();
		if (msgLen >= 1) {
			uint8_t hdr = msg[0];
			if (hdr == 0x01 && msgLen == 3) {
				memcpy(&coreState.avgSpeed, msg + 1, 2);
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
		
		if (!(PIN_BTN & _BV(B_BTN))) {
			robotMode = (robotMode + 1) % 3;
			_delay_ms(1000);
		} 
		
		switch (robotMode) {
			case STOP:
				PORT_LED0 &= ~_BV(B_LED0);
				PORT_LED1 &= ~_BV(B_LED1);
				PORT_LED2 |= _BV(B_LED2);
				break;
			case FOLLOW:
				PORT_LED0 &= ~_BV(B_LED0);
				PORT_LED1 |= _BV(B_LED1);
				PORT_LED2 &= ~_BV(B_LED2);
				break;
			case STRAIGHT:
				PORT_LED0 &= ~_BV(B_LED0);
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
    DDRD |= _BV(B_LED2);
    DDRC |= _BV(B_LED1) | _BV(B_LED0);
	//output for onboard LED
	DDRD |= _BV(5);
	
	//pullup resistors for  buttons
	PORT_BTN |= _BV(B_BTN);
	
	//buzzer
	DDRD |= _BV(B_BZR);
}