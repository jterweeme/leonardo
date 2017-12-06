/*
PS-DAT: D8/PB4
PS-CLK: SDA/PD1/INT1
*/

#include "stream.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define F_CPU 16000000UL
#include <util/delay.h>
#include "keyboard.h"

PS2Keyboard *g_kb;

ISR(TIMER0_OVF_vect)
{
    g_kb->timeTick();
}

ISR(INT1_vect)
{
    g_kb->isr();
}

inline char nibble(uint8_t n)
{
    return n <= 9 ? '0' + n : 'A' + n - 10;
}

int main()
{
    USBStream usb;
    usb.writeString("Keyboard Test:\r\n");
    usb.flush();
    sei();
    TCCR0A = 0;
    TCCR0B = 1<<CS01 | 1<<CS00;     // clk/64
    TCNT0 = 0;
    TIMSK0 = 1<<TOIE0;              // timer0 overflow generates interrupt
    EICRA |= 1<<ISC11;  // ext int 1 falling edge
    EIMSK |= 1<<INT1;   // enable int1
    PS2Keyboard keyboard;
    g_kb = &keyboard;

    while (true)
    {
        uint8_t x = keyboard.get_scan_code();

        if (x > 0)
        {
            usb.write(nibble(x >> 4));
            usb.write(nibble(x & 0xf));
            usb.writeString("\r\n");
            usb.flush();
        }
    }

    return 0;
}




