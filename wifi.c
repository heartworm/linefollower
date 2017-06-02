#include "wifi.h"

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
			if (bufferFull()) bufferTrash();
			bufferPush(in);
			
			if (in == '\r') {
				inCr = true;
			}
			
			if (inCr && in == '\n') {
				inCr = false;
				bufferPush(0x00);
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
	switch (wifiState){
		case INIT:
			if (strcmp(rxBuf,"OK\r\n") == 0) {
				uart_puts("AT+CIPSTART=\"UDP\",\"192.168.69.2\",12345\r\n");
				wifiState = ALIVE;
			}
			break;
		case ALIVE:
			if (strcmp(rxBuf,"OK\r\n") == 0) {
				uart_puts("AT+CIPSTATUS\r\n");
				wifiState = CHECK_CONNECTION;
			}
			break;
		case FINDING_PORT:
			const statString = "+CIPSTATUS=0,\"UDP\",\"192.168.69.2\",12345,";
			if (strstr(rxBuf, statString) == rxBuf) {
				uart_puts("AT+CIPSEND=1");
				wifiState = SENDPING;
			}
			break;
		case SENDPING:
			if (rxBuf[0] == '>') {
				uart_puts('a\r\n');
			}
			break;
		}
	}
	bufferTrash();
}