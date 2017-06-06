#ifndef WIFI_H
#define WIFI_H

#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <string.h>


enum WifiState {
	INIT,
	ALIVE,
	CONNECTING,
	GOT_PORT,
	PROMPT,
	SEND,
	ERROR,
	SENDWAIT,	
} wifiState;

bool inCr;

void setupWifi();
void setupSerial();
void serialRecv();
void wifiProcess();

void bufferTrash();

uint8_t bufferFull();

uint8_t bufferEmpty();

void bufferPush(uint8_t in);

uint8_t rxBuf[32]; 
uint8_t msgBuf[32];
char *sendBuf;
uint8_t rxNew;

#endif