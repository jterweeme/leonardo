#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_
#include <avr/pgmspace.h>
#include "misc.h"

class MyBitset
{
    uint8_t _bitcount = 0, _incoming = 0;
public:
    inline void reset() { _bitcount = _incoming = 0; }
    void addBit(uint8_t val);
    uint8_t count() const { return _bitcount; }
    uint8_t incoming() const { return _incoming; }
};

class PS2Keyboard
{
private:
    MyBitset _bitset;
    CircBuf _buf;
    uint32_t _timeTicks = 0;
    uint32_t _prevTicks = 0;
public:
    PS2Keyboard();
    void timeTick() { _timeTicks++; }
    void isr();
    void ps2interrupt();
    uint8_t get_scan_code() { return _buf.get(); }
};

#endif


