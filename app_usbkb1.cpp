#include "usbkb.h"
#include "misc.h"

int main()
{
    Serial serial;
    serial.init();
    USBKB kb;
    
    while (true)
        kb.usbTask();

    return 0;
}


