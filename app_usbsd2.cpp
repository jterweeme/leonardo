#include "usbsd2.h"
#include <avr/interrupt.h>

int main(void)
{
#ifdef DEBUG
    Serial serial;
    g_serial = &serial;
    serial.init();
    serial.write("onzin\r\n");
#endif
    USBSD usbsd;
    sei();  // dit is nodig!

	for (;;)
	{
        usbsd.MassStorage_Task();
	}
}






