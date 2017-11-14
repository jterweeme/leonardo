#include "busby.h"

inline char nibble(uint8_t n)
{
    return n <= 9 ? '0' + n : 'A' + n - 10;
}

int main()
{
    USB usb;

    while (true)
    {
        for (uint32_t i = 0; i < 0xffffffff; i++)
        {
            for (int8_t j = 7; j >= 0; j--)
                usb.myPutc(nibble(i >> (j << 2) & 0xf));

            usb.myPutc('\r');
            usb.myPutc('\n');
            usb.flush();           
        }
    }

    return 0;
}


