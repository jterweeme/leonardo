CC = avr-g++ --std=c++11 -mmcu=atmega32u4
CMD = $(CC) -O2 -c -Wall -Wno-unused-variable -Wno-parentheses
APP = app_vgacalc1.elf

%.o: %.cpp
	$(CMD) $<

%.elf: %.o
	$(CC) -o $@ $^

all: app_calc1.elf \
    app_calc2.elf \
    app_hellousb1.elf \
    app_mouse1.elf \
    app_ostream1.elf \
    app_potmeter1.elf \
    app_ps2kb2.elf \
    app_sdinfo1.elf \
    app_sdod1.elf \
    app_sdod2.elf \
    app_tone1.elf \
    app_uartloop1.elf \
    app_usbkb1.elf \
    app_usbkb2.elf \
    app_usbkb3.elf \
    app_usbkb4.elf \
    app_usbloop1.elf \
    app_usbsd2.elf \
	app_vga1.elf \
    app_vgacalc1.elf

app_calc1.elf: app_calc1.o calc.o tft.o misc.o button.o analog.o cdc.o busby.o
app_calc2.elf: app_calc2.o calc.o cdc.o busby.o
app_hellousb1.elf: app_hellousb1.o cdc.o busby.o misc.o
app_mouse1.elf: app_mouse1.o ps2mouse.o cdc.o busby.o misc.o
app_ostream1.elf: app_ostream1.o cdc.o busby.o misc.o
app_potmeter1.elf: app_potmeter1.o analog.o cdc.o busby.o
app_ps2kb2.elf: app_ps2kb2.o cdc.o busby.o keyboard.o misc.o leonardo.o
app_sdinfo1.elf: app_sdinfo1.o busby.o cdc.o zd2card.o leonardo.o
app_sdod1.elf: app_sdod1.o zd2card.o cdc.o busby.o leonardo.o misc.o
app_sdod2.elf: app_sdod2.o zd2card.o cdc.o busby.o leonardo.o misc.o
app_tone1.elf: app_tone1.o
app_uartloop1.elf: app_uartloop1.o misc.o
app_usbkb1.elf: app_usbkb1.o usbkb.o busby.o misc.o
app_usbkb2.elf: app_usbkb2.o usbkb.o keyboard.o busby.o misc.o leonardo.o
app_usbkb3.elf: app_usbkb3.o
app_usbkb4.elf: app_usbkb4.o usbkb2.o busby2.o
app_usbloop1.elf: app_usbloop1.o cdc.o busby.o misc.o
app_usbsd2.elf: app_usbsd2.o usbsd.o busby.o misc.o zd2card.o leonardo.o
app_vga1.elf: app_vga1.o vga.o
app_vgacalc1.elf: app_vgacalc1.o vga.o keyboard.o misc.o calc.o leonardo.o

analog.o: analog.cpp analog.h
app_calc1.o: app_calc1.cpp button.h analog.h glcdfont.h
app_calc2.o: app_calc2.cpp stream.h
app_hellousb1.o: app_hellousb1.cpp
app_mouse1.o: app_mouse1.cpp ps2mouse.h stream.h
app_ostream1.o: app_ostream1.cpp stream.h
app_potmeter1.o: app_potmeter1.cpp analog.h
app_ps2kb2.o: app_ps2kb2.cpp
app_sdinfo1.o: app_sdinfo1.cpp zd2card.h stream.h board.h leonardo.h
app_sdod1.o: app_sdod1.cpp zd2card.h
app_sdod2.o: app_sdod2.cpp zd2card.h
app_tone1.o: app_tone1.cpp
app_uartloop1.o: app_uartloop1.cpp misc.h
app_usbkb1.o: app_usbkb1.cpp usbkb.h
app_usbkb2.o: app_usbkb2.cpp
app_usbkb3.o: app_usbkb3.cpp
app_usbkb4.o: app_usbkb4.cpp
app_usbloop1.o: app_usbloop1.cpp
app_usbsd2.o: app_usbsd2.cpp usbsd.h busby.h
app_vga1.o: app_vga1.cpp vga.h screenFont.h
app_vgacalc1.o: app_vgacalc1.cpp vga.h screenFont.h keyboard.h calc.h
busby.o: busby.cpp busby.h
button.o: button.cpp button.h
calc.o: calc.cpp calc.h
cdc.o: cdc.cpp cdc.h busby.h
keyboard.o: keyboard.cpp keyboard.h
leonardo.o: leonardo.cpp leonardo.h
misc.o: misc.cpp
ps2mouse.o: ps2mouse.cpp ps2mouse.h
tft.o: tft.cpp
usbkb.o: usbkb.cpp usbkb.h
usbkb2.o: usbkb2.cpp
usbsd.o: usbsd.cpp usbsd.h busby.h
vga.o: vga.cpp vga.h leonardo.h
zd2card.o: zd2card.cpp zd2card.h

download:
	avrdude -c stk500 -p m32u4 -P /dev/ttyUSB0 -U $(APP)

clean:
	rm -vf *.o *.elf

rebuild: clean all


