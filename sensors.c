#include "sensors.h"

const uint16_t centerValue = (LINE_SENSORS-1) * 500;
const uint8_t MUXES[] = {MUX_LIN1, MUX_LIN2, MUX_LIN3, MUX_LIN4, MUX_LIN5, MUX_LC, MUX_RC};

volatile uint16_t readings[] = {0, 0, 0, 0, 0, 0, 0}; //10 bit output from the ADC
volatile uint32_t adjusted[] = {0, 0, 0, 0, 0, 0, 0}; //10 bit output from the ADC
volatile uint16_t maxes[] = {0, 0, 0, 0, 0, 767, 767}; //10 bit output from the ADC
volatile uint16_t mins[] = {1023, 1023, 1023, 1023, 1023, 255, 255}; //10 bit output from the ADC
volatile uint8_t curMux = 0;

bool calibrating = true;
uint16_t lastPos = (LINE_SENSORS-1) * 500;
bool lostLine = false;

void setupADC() {
	ADMUX = _BV(REFS0); //Internal Vcc reference with a cap at AREF
	setMux(curMux); //look at the first sensor
	ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
	// enable ADC with interupts, cpu speed/128 is our ADC clock
	ADCSRA |= _BV(ADSC); 
}

void setMux(int mux) { 
	//check datasheet for ADC Mux values. This changes the sensor that is being read. 
	ADMUX = (ADMUX & 0b11100000) | (MUXES[mux] & 0b00011111);
	ADCSRB = (ADCSRB & ~(_BV(MUX5))) | (MUXES[mux] & _BV(MUX5));
}


ISR(ADC_vect) {
	uint16_t reading = ADC;
	readings[curMux] = reading; //save our reading in the appropriate array bin
	if(calibrating) { //do the calibration here so we can use it in the main code
		if (reading < mins[curMux]) mins[curMux] = reading;
		if (reading > maxes[curMux]) maxes[curMux] = reading;
	}
	
	//focus on the next sensor, if at the last one, loop around
	curMux = (curMux + 1) % ADC_NUMBER;
	setMux(curMux);
	ADCSRA |= _BV(ADSC); //start new conversion
}

uint32_t getAdjReading(uint8_t adcIndex) {
	//TODO change to uint16_T 
	//get the max and min expected values for each sensor (written in the interrupt)
	//the difference is the usable range of the sensor. 
	uint32_t range = maxes[adcIndex] - mins[adcIndex];
	if (range == 0) return 0; //divide by 0 avoidance
	
	//where in between the minimum and maximum sensor values is the latest reading?
	//express this as a proportion between 0 and 1000
	//ideally 1000 is the line and 0 is the black surface, ofc this is wishful thinking
	//the bot usually gets like 200 and 800 which is pretty good
	uint32_t adjReading = (((uint32_t)readings[adcIndex] - mins[adcIndex]) * 1000);
	adjReading = adjReading / (range);
	return adjReading;
}

uint16_t getCoL() {
	uint32_t massDist = 0;
	uint32_t mass = 0;
	uint16_t spreadMin = 1000;
	uint16_t spreadMax = 0;
	
	//floats have been replaced with 1000 int precision
	//for each sensor
	for (uint8_t i = 0; i < LINE_SENSORS; i++) {
		
		// get the max and min expected values for each sensor (written in the interrupt)
		// the difference is the usable range of the sensor. 
		// uint32_t range = maxes[i] - mins[i];
		// if (range == 0) return centerValue; //divide by 0 avoidance
		
		// where in between the minimum and maximum sensor values is the latest reading?
		// express this as a proportion between 0 and 1000
		// ideally 1000 is the line and 0 is the black surface, ofc this is wishful thinking
		// the bot usually gets like 200 and 800 which is pretty good
		// uint32_t adjReading = (((uint32_t)readings[i] - mins[i]) * 1000);
		// adjReading = adjReading / (range);
		
		uint32_t adjReading = getAdjReading(i);
		adjReading = adjReading == 0 ? 1 : adjReading;
		
		//calculating the minimum and maximum sensor values at this time for lost line situation
		if (adjReading < spreadMin) spreadMin = adjReading;
		if (adjReading > spreadMax) spreadMax = adjReading;

		adjusted[i] = adjReading; //not necessary array but useful for debugging
		
		//think of brightnesses as mass and each sensor as having a displacement
			//and you'll understand how i've abused a sorta-center-of-mass equation to find the line
		mass += adjReading;
		massDist += adjReading * (i * 1000);
	}
	
	//if the line was last left of center, then steer hard left if we lose the line, and vice versa
	uint16_t lastDir = lastPos < centerValue ? 0 : (LINE_SENSORS-1)*1000;
	// if there is very little variation in the data
		//and the overall field is relatively dark (doesn't trigger on cross intersections)
		//we've probably lost the line
	uint16_t valueSpread = spreadMax - spreadMin;
	const uint16_t LOST_THRESH = 300;
	const uint16_t FOUND_THRESH = 350;
	const uint16_t DARKNESS_THRESH = ((1000 * LINE_SENSORS) / 2); // in terms of sum of all readings
	
	if (valueSpread < LOST_THRESH && mass < DARKNESS_THRESH) { 
		lostLine = true;
	}  else if (valueSpread > FOUND_THRESH) { //some hysteresis to stop retarded twitching, commit to a direction. 
		lostLine = false;
	}
	
	if (lostLine) return lastDir;
	
	lastPos = (uint16_t)(massDist / mass);
	return lastPos;
}

uint32_t getRightCornerVal() {
	return getAdjReading(IND_RC);
}

uint32_t getLeftCornerVal() {
	return getAdjReading(IND_LC);
}

float toLineError(uint16_t center) {
	float error = (center - 2000.0) / 2000.0;
	return error;
}

bool isLineLost() {
    return lostLine;
}