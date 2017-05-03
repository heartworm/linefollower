#include "serial.h"

uint8_t rxNew = 0;
bool inFrame = 0;
bool inEsc = 0;

void setupSerial() {
	uart_init(UART_BAUD_SELECT(57600, 16000000UL));
}

uint8_t serialRecv() {
	uint16_t statusIn = uart_getc();
	uint16_t status = statusIn & 0xFF00;
	uint8_t in = statusIn & 0x00FF;
	
	uint8_t packetReady = 0;
	
	if (status == 0) {
		if (bufferFull()) bufferTrash();
		
		if (!inFrame) inEsc = false;	
		if (!inEsc) {
			if (in == FLG_ESC && inFrame) {
				inEsc = true;
			} else if (in == FLG_STX) {
				bufferTrash();
				inFrame = true;
			} else if (in == FLG_ETX) {
				if (inFrame) packetReady = rxNew;
				bufferTrash();
			} else {
				if (inFrame) bufferPush(in);
			}
		} else {
			inEsc = false;
			if (inFrame) bufferPush(in);
		}
	} else if (status != UART_NO_DATA) {
		bufferTrash();
	}
	
	return packetReady;
}

uint8_t bufferFull() {
	return rxNew >= RX_BUFLEN;
}

uint8_t bufferEmpty() {
	return rxNew <= 0;
}

void bufferPush(uint8_t in) {
	rxBuf[rxNew++] = in;
} 

uint8_t *serialGetMsgBuffer() {
	return rxBuf;
}

void bufferTrash() {
	rxNew = 0;
	inEsc = false;
	inFrame = false;
}

void serialSendEscaped(uint8_t *data, uint8_t len) {	
	uint8_t outData[(len * 2) + 2];
	uint16_t pos = 0;
	outData[pos++] = FLG_STX;
	for (uint8_t i = 0; i < len; i++) {
		if (data[i] == FLG_ESC || data[i] == FLG_STX || data[i] == FLG_ETX) {
			outData[pos++] = FLG_ESC;
		}
		outData[pos++] = data[i];
	}
	outData[pos++] = FLG_ETX;
	serialSend(outData, pos);
}

void serialSend(uint8_t *data, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		uart_putc(data[i]);
	}
}