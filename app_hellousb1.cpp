#include "cdc.h"
#include "misc.h"

inline char nibble(uint8_t n)
{
    return n <= 9 ? '0' + n : 'A' + n - 10;
}

int main()
{
    Serial serial;
    serial.init();
    Serial::instance->write("main\r\n");
    CDC usb;

    while (true)
    {
        for (uint32_t i = 0; i < 0xffffffff; i++)
        {
            for (volatile uint32_t i = 0; i < 0xfffff; i++)
                ;

            for (int8_t j = 7; j >= 0; j--)
                usb.sendByte(nibble(i >> (j << 2) & 0xf));

            usb.sendByte('\r');
            usb.sendByte('\n');
            usb.flush();
            Serial::instance->write("Debug bericht\r\n");
        }
    }

    return 0;
}


