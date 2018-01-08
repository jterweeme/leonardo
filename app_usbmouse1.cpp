#include "usbmouse.h"
#include <avr/interrupt.h>

int main()
{
    USBMouse usbmouse;
    sei();
    DDRC |= 1<<7;

    while (true)
    {
        
        MouseReportData mrd;
        usbmouse.sendReport(mrd);

        for (volatile uint32_t i = 0; i < 0xf; i++)
            ;
    }

    return 0;
}


