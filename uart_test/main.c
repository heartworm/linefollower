#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h> 
#include <util/delay.h>    
#include <stdint.h>       
#include <stdbool.h>    
#include <stdio.h>      
#include <stdlib.h>
#include <math.h>     

#include "wifi.h"

void main() {
	DDRD |= _BV(1);
	setupSerial();
	setupWifi();
	while (1) {
		serialRecv();
	}
}
