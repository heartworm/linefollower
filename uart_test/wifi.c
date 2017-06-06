#include "wifi.h"

uint8_t rxNew = 0;
bool inCr = false;



void setupSerial() {
	uart_init(UART_BAUD_SELECT(115200, 16000000UL));
}

void setupWifi() {
	uart_puts("AT\r\n");
	
}

void serialRecv() {
	uint16_t status;
	do 	{
		uint16_t statusIn = uart_getc();
		uint16_t status = statusIn & 0xFF00;
		uint8_t in = statusIn & 0x00FF;
		
		if (status == 0) {	
			PORTD |= _BV(1);
			if (bufferFull()) bufferTrash();
			bufferPush(in);
			
			if (wifiState == SENDWAIT && in == '>') {
				wifiState = SEND;
				wifiProcess();
			} if (in == '\r') {
				inCr = true;
			} else if (inCr && in == '\n') {
				inCr = false;
				wifiProcess();
			} else if (inCr) {
				inCr = false;
			}
			
		} else if (status != UART_NO_DATA) {
			bufferTrash();
		}
		
		
	} while (status == 0);
}

void wifiProcess() {
	bool isOk = false;
	if (strcmp(rxBuf, "OK\r\n") == 0) {
		isOk = true;
	}
	
	switch (wifiState){
		case INIT:
			if (isOk) {
				uart_puts("AT+CIPSTART=\"UDP\",\"192.168.43.10\",8888\r\n");
				wifiState = ALIVE;
			}
			break;
		case ALIVE:
			if (isOk) {
				uart_puts("AT+CIPSEND=5");
				sendBuf = "aaaaa";
				wifiState = SENDWAIT;
			}
			break;
		case SEND:
			uart_puts(sendBuf);
			uart_puts("\r\n");
			wifiState = PROMPT;
			break;
	}
	bufferTrash();	
	isOk = false;
}


uint8_t bufferFull() {
	return rxNew >= 32;
}

uint8_t bufferEmpty() {
	return rxNew <= 0;
}

void bufferPush(uint8_t in) {
	rxBuf[rxNew++] = in;
} 

void bufferTrash() {
	rxNew = 0;
	inCr = false;
}