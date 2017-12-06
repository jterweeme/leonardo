#include "ps2mouse.h"
#include "stream.h"
#include <stdio.h>

int main()
{
    USBStream usb;
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
        usb.writeString(buf);
        
    }

    return 0;
}


