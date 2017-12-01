#include <avr/io.h>
#include <avr/sleep.h>

int main()
{
    DDRB = 0xff;
    DDRC = 0xff;
    DDRD = 0xff;
    OCR0A = 100;
    TCCR0A |= 1<<WGM01 | 1<<COM0A0;
    TCCR0B |= 1<<CS01 | 1<<CS00;
    TCCR4A |= 1<<WGM41 | 1<<COM4A0;
    TCCR4B |= 1<<CS41 | 1<<CS40;

    while (true)
        sleep_mode();

    return 0;
}



