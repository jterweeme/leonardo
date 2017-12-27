#include "usbsd.h"
#include "misc.h"

int main()
{
    Serial serial;
    *p_ubrr1 = 102;
    *p_ucsr1b = 1<<txen1;
    USBSD sd;
    Serial::instance->write("main()\r\n");

    while (true)
    {
        sd.msTask();
        sd.usbTask();
    }

    return 0;
}


