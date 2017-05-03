MAIN = main

FLASH.bin : $(MAIN).elf
	avr-objcopy -O binary $(MAIN).elf FLASH.bin

$(MAIN).elf : $(MAIN).o lcd.o motors.o sensors.o utils.o encoders.o pid.o serial.o uart.o
	avr-gcc -mmcu=atmega32u4 $(MAIN).o lcd.o motors.o sensors.o utils.o encoders.o pid.o serial.o uart.o -o $(MAIN).elf

$(MAIN).o : $(MAIN).c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c $(MAIN).c -o $(MAIN).o
    
lcd.o : lcd.c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c lcd.c -o lcd.o

motors.o : motors.c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c motors.c -o motors.o
    
sensors.o : sensors.c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c sensors.c -o sensors.o
	
utils.o : utils.c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c utils.c -o utils.o
	
encoders.o : encoders.c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c encoders.c -o encoders.o
	
pid.o : pid.c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c pid.c -o pid.o
	
serial.o : serial.c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c serial.c -o serial.o
	
uart.o : uart.c
	avr-gcc -DF_CPU=16000000UL -O1 -mmcu=atmega32u4 -std=c99 -c uart.c -o uart.o