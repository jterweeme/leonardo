/*
PS2 Mouse

PS2CLK: D3
PS2DAT: D2
*/

#include "ps2mouse.h"
#include "stream.h"
#include <stdio.h>

#define F_CPU 16000000UL
#include <util/delay.h>

int main()
{
    CDC cdc;
    USBStream cout(&cdc);
    PS2Mouse mouse;
    mouse.write(0xff);
    mouse.read();
    mouse.read();
    mouse.read();
    mouse.write(0xf0);
    mouse.read();
    _delay_us(100);

    while (true)
    {
        mouse.write(0xeb);
        mouse.read();
        uint8_t mstat = mouse.read();
        uint8_t mx = mouse.read();
        uint8_t my = mouse.read();
        char buf[50];
        snprintf(buf, 50, "%u %u\r\n", mx, my);
        cout.writeString(buf);
        cout.flush();
    }

    return 0;
}


