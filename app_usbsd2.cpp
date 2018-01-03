#include "usbsd2.h"

int main(void)
{
#ifdef DEBUG
    Serial serial;
    g_serial = &serial;
    serial.init();
    serial.write("onzin\r\n");
#endif
    USBSD usbsd;
    sei();

	for (;;)
	{
        usbsd.MassStorage_Task();
	}
}






