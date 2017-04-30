#ifndef SERIAL_H
#define SERIAL_H

#define RX_BUFLEN 16
#define FLG_ESC 0xFD
#define FLG_ETX 0xFE
#define FLG_STX 0xFF


#include <avr/io.h>          // register definitions
#include <stdbool.h>    

#include "uart.h"       

uint8_t rxNew;
bool inFrame;
bool inEsc;
uint8_t rxBuf[RX_BUFLEN];

void setupSerial();
uint8_t serialRecv();

uint8_t bufferFull();

uint8_t bufferEmpty();

uint8_t bufferNext();

void bufferPush(uint8_t in);


void bufferTrash();
void serialSendEscaped(uint8_t *data, uint8_t len);
void serialSend(uint8_t *data, uint16_t len);
uint8_t *serialGetMsgBuffer();

#endif