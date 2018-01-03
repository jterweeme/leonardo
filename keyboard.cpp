/*
PS2 CLK: SDA
*/

#include "keyboard.h"
#include <avr/io.h>

void MyBitset::addBit(uint8_t val)
{
    if (_bitcount - 1 <= 7)
        _incoming |= val << _bitcount - 1;

    _bitcount++;
}

PS2Keyboard::PS2Keyboard(Pin *dat) : _dat(dat)
{
    _dat->direction(INPUT);
    _dat->set();
    DDRD &= ~(1<<DDD1 | 1<<DDD0);
    PORTD |= 1<<1 | 1<<0;
}

void PS2Keyboard::isr()
{
    bool val = _dat->read();

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



