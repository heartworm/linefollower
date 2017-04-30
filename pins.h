#ifndef PINS_H
#define PINS_H

#define B_LED0 7
#define PORT_LED0 PORTC
#define B_LED1 7
#define PORT_LED1 PORTD
#define B_LED2 6
#define PORT_LED2 PORTC

//motor driver outputs
#define B_MA1 5
#define PORT_MA1 PORTB
#define B_MA2 6
#define PORT_MA2 PORTB
#define B_MB2 7
#define PORT_MB2 PORTB 
#define B_MB1 0
#define PORT_MB1 PORTD

#define ADC_NUMBER 7
#define LINE_SENSORS 5
#define MUX_RC 0b10011 //adc11 PB4
#define MUX_LC 0b00000 //adc0 PF0
#define MUX_LIN1 0b000001 //adc1  PF1
#define MUX_LIN2 0b000100 //adc4  PF4
#define MUX_LIN3 0b000101 //adc5  PF5
#define MUX_LIN4 0b000110 //adc6  PF6
#define MUX_LIN5 0b000111 //adc7 PF7

#define IND_RC 6
#define IND_LC 5
#define IND_LIN5 4
#define IND_LIN4 3
#define IND_LIN3 2
#define IND_LIN2 1
#define IND_LIN1 0

#define POS_LIN1 0
#define POS_LIN2 1000
#define POS_LIN3 2000
#define POS_LIN4 3000
#define POS_LIN5 4000

#endif