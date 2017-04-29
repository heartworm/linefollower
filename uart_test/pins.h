#ifndef PINS_H
#define PINS_H

#define B_LED0 7
#define PORT_LED0 PORTC
#define B_LED1 7
#define PORT_LED1 PORTD
#define B_LED2 6
#define PORT_LED2 PORTC

#define PORT_BTN PORTD
#define PIN_BTN PIND
#define B_BTN1 6
#define B_BTN0 4


//motor driver outputs
#define B_MA1 5
#define PORT_MA1 PORTB
#define B_MA2 6
#define PORT_MA2 PORTB
#define B_MB2 7
#define PORT_MB2 PORTB 
#define B_MB1 0
#define PORT_MB1 PORTD

#define MA1 0
#define MA2 1
#define MB1 2
#define MB2 3

#define ADC_NUMBER 5
#define LINE_SENSORS 5

#define MUX_LIN5 0b000100 //adc4  PF4
#define MUX_LIN4 0b000101 //adc5  PF5
#define MUX_LIN3 0b000110 //adc6  PF6
#define MUX_LIN2 0b000111 //adc7  PF7
#define MUX_LIN1 0b100011 //adc11 PB4

#define POS_LIN1 0
#define POS_LIN2 1000
#define POS_LIN3 2000
#define POS_LIN4 3000
#define POS_LIN5 4000

#endif