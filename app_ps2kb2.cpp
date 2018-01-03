/*
PS-DAT: D8/PB4
PS-CLK: SDA/PD1/INT1
*/

#include "stream.h"
#include <avr/interrupt.h>
#include "keyboard.h"

#include "misc.h"

PS2Keyboard *g_kb;

ISR(TIMER0_OVF_vect)
{
    g_kb->timeTick();
}

ISR(INT0_vect)
{
    g_kb->isr();
}

inline char nibble(uint8_t n)
{
    return n <= 9 ? '0' + n : 'A' + n - 10;
}

int main()
{
    sei();
    TCCR0A = 0;
    TCCR0B = 1<<CS01 | 1<<CS00;     // clk/64
    TCNT0 = 0;
    TIMSK0 = 1<<TOIE0;              // timer0 overflow generates interrupt
    EICRA |= 1<<ISC00 | 1<<ISC01;  // ext int 0 falling edge
    EIMSK |= 1<<INT0;   // enable int0
    Board b;
    PS2Keyboard keyboard(&b.pin8);
    g_kb = &keyboard;
    CDC cdc;
    USBStream os(&cdc);
    os.writeString("Keyboard Test:\r\n");

    while (true)
    {
        uint8_t x = keyboard.get_scan_code();

        if (x > 0)
        {
            os.write(nibble(x >> 4));
            os.write(nibble(x & 0xf));
            os.writeString("\r\n");
            os.flush();
        }
    }

    return 0;
}




