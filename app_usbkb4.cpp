#include <avr/io.h>
#include "usbkb2.h"

int main()
{
    DDRF &= ~(1<<0 | 1<<1 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    PORTF |= 1<<0 | 1<<1 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
    USBKB kb;

	while (true)
	{
        kb.USBTask();
	}
}





