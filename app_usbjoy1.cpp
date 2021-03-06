/*
USB Joystick using analog knob and buttons

Analog knob x axis: A5/ADC0
Analog knob y axis: A4/ADC1
Button 0: A0/PF7
Button 1: A1/PF6
*/

#include "usbjoy.h"
#include "analog.h"
#include <avr/interrupt.h>

int main()
{
    USBJoy joy;
    sei();
    DDRF &= ~(1<<7 | 1<<6);
    PORTF |= 1<<7 | 1<<6;
    JoyReportData jrd;
    Analog analog;
    analog.init();

    while (true)
    {
        jrd.x = jrd.y = jrd.z = 0;
        jrd.button = 0;

        uint16_t reading = analog.read(Analog::ADC0);
        jrd.x = (reading >> 2) - 128;
        reading = analog.read(Analog::ADC1);
        jrd.y = (reading >> 2) - 128;

        if ((PINF & 1<<7) == 0)
            jrd.button |= 1<<0;
        
        if ((PINF & 1<<6) == 0)
            jrd.button |= 1<<1;

        joy.sendReport(jrd);
    }

    return 0;
}



