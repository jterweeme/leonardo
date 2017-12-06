#include "cdc.h"

inline char nibble(uint8_t n)
{
    return n <= 9 ? '0' + n : 'A' + n - 10;
}

int main()
{
    CDC usb;

    while (true)
    {
        for (uint32_t i = 0; i < 0xffffffff; i++)
        {
            for (int8_t j = 7; j >= 0; j--)
                usb.sendByte(nibble(i >> (j << 2) & 0xf));

            usb.sendByte('\r');
            usb.sendByte('\n');
            usb.flush();           
        }
    }

    return 0;
}


