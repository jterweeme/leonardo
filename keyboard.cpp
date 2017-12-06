#include "keyboard.h"
#include <avr/interrupt.h>

void MyBitset::addBit(uint8_t val)
{
    if (_bitcount - 1 <= 7)
        _incoming |= val << _bitcount - 1;

    _bitcount++;
}

PS2Keyboard::PS2Keyboard()
{
    DDRB &= ~(1<<DDB4);
    PORTB |= 1<<PORTB4;
    DDRD &= ~(1<<DDD1);
    PORTD |= 1<<PORTD1;
}

void PS2Keyboard::isr()
{
    uint8_t val = PINB & 1<<4;

    if (_timeTicks - _prevTicks > 10)
        _bitset.reset();

    _prevTicks = _timeTicks;
    _bitset.addBit(val);

    if (_bitset.count() == 11)
    {
        _buf.push(_bitset.incoming());
        _bitset.reset();
    }
}



