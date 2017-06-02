#ifndef WIFI_H
#define WIFI_H

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

void setupWifi();
void wifiSend();
void wifiProcess();
uint8_t wifiRecv();
uint8_t *wifiGetMsg();

uint8_t recvBuf[32]; 
uint8_t msgBuf[32];


#endif