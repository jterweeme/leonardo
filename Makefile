CC = avr-g++ --std=c++11 -mmcu=atmega32u4
CMD = $(CC) -O2 -c
APP = app_vgacalc1.elf

%.o: %.cpp
	$(CMD) $<

%.elf: %.o
	$(CC) -o $@ $^

all: app_hellousb1.elf \
    app_tone1.elf \
    app_usbloop1.elf \
	app_vga1.elf \
    app_vgacalc1.elf

app_hellousb1.elf: app_hellousb1.o misc.o busby.o
app_tone1.elf: app_tone1.o
app_usbloop1.elf: app_usbloop1.o busby.o misc.o
app_vga1.elf: app_vga1.o vga.o
app_vgacalc1.elf: app_vgacalc1.o vga.o keyboard.o misc.o calc.o busby.o

app_hellousb1.o: app_hellousb1.cpp busby.h
app_tone1.o: app_tone1.cpp
app_usbloop1.o: app_usbloop1.cpp
app_vga1.o: app_vga1.cpp vga.h screenFont.h
app_vgacalc1.o: app_vgacalc1.cpp vga.h screenFont.h keyboard.h calc.h
busby.o: busby.cpp busby.h
calc.o: calc.cpp calc.h
keyboard.o: keyboard.cpp keyboard.h
misc.o: misc.cpp
vga.o: vga.cpp vga.h leonardo.h

download:
	avrdude -c stk500 -p m32u4 -P /dev/ttyUSB0 -U $(APP)

clean:
	rm -vf *.o *.elf

rebuild: clean all


