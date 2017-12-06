#include "ps2mouse.h"

#define F_CPU 16000000UL
#include <util/delay.h>

void PS2Mouse::write(uint8_t data)
{
    uint8_t parity = 1;
    DDRD &= ~(1<<1);    //data go high
    PORTD |= 1<<1;
    DDRD &= ~(1<<0);    //clk go high
    PORTD |= 1<<0;
    _delay_us(300);
    DDRD |= 1<<0;       //clk
    PORTD &= ~(1<<0);
    _delay_us(300);
    DDRD |= 1<<1;       //data
    PORTD &= ~(1<<1);
    _delay_us(10);
    DDRD &= ~(1<<0);    //clk
    PORTD |= 1<<0;
    
    while (PIND & 1<<0)
        ;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & 1)
        {
            DDRD &= ~(1<<1);
            PORTD |= 1<<1;
        }
        else
        {
            DDRD |= 1<<1;
            PORTD &= ~(1<<1);
        }

        while ((PIND & 1<<0) == 0)
            ;

        while ((PIND & 1<<0) == 1)
            ;

        parity = parity ^ (data & 1);
        data = data >> 1;
    }

    if (parity)
    {
        DDRD &= ~(1<<1);    // data go hi
        PORTD |= 1<<1;
    }
    else
    {
        DDRD |= 1<<1;       // data go low
        PORTD &= ~(1<<1);
    }

    while ((PIND & 1<<0) == 0)
        ;

    while ((PIND & 1<<0) == 1)
        ;

    DDRD &= ~(1<<1);
    PORTD |= 1<<1;
    _delay_us(50);

    while ((PIND & 1<<0) == 1)
        ;

    while (((PIND & 1<<0) == 0) || ((PIND & 1<<1) == 0))
        ;

    DDRD |= 1<<0;
    PORTD &= ~(1<<0);
}

uint8_t PS2Mouse::read()
{
    uint8_t data = 0;
    uint8_t bit = 1;

    DDRD &= ~(1<<0);    //clk go hi
    PORTD |= 1<<0;
    DDRD &= ~(1<<1);    // data go hi
    PORTD |= 1<<1;
    _delay_ms(50);
    
    while ((PIND & 1<<0) == 1)
        ;

    _delay_ms(5);

    while ((PIND & 1<<0) == 0)
        ;

    for (uint8_t i = 0; i < 8; i++)
    {
        while ((PIND & 1<<0) == 1)
            ;

        if ((PIND & 1<<1) == 1)
            data = data | bit;

        while ((PIND & 1<<0) == 0)
            ;

        bit = bit << 1;
    }

    while ((PIND & 1<<0) == 1)   // clk high?
        ;

    while ((PIND & 1<<0) == 0)
        ;

    while ((PIND & 1<<0) == 1)
        ;

    while ((PIND & 1<<0) == 0)
        ;

    DDRD |= 1<<0;
    PORTD &= ~(1<<0);
    return data;
}



