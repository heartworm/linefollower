#ifndef WIFI_H
#define WIFI_H

#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

const char EOL[] = "\r\n";

enum WifiState {
	INIT,
	ALIVE,
	CONNECTING,
	GOT_PORT,
	PROMPT,
	SEND,
	ERROR,
	
	
} wifiState;

bool inCr = false;
bool inSendPrompt = false;

void setupWifi();
void serialRecv();
void wifiProcess();

uint8_t recvBuf[32]; 
uint8_t msgBuf[32];
uint8_t sendBuf[32];
uint8_t rxNew = 0;

#endif