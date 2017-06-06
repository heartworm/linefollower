MAIN = main

FLASH.bin : $(MAIN).elf
	avr-objcopy -O binary $(MAIN).elf FLASH.bin

$(MAIN).elf :
	avr-gcc -O2 -mmcu=atmega32u4 $(MAIN).c uart.c wifi.c -o $(MAIN).elf