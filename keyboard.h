#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_
#include "misc.h"

static constexpr uint8_t
    PS2_A = 0x1c,
    PS2_B = 0x32,
    PS2_C = 0x21,
    PS2_D = 0x23,
    PS2_E = 0x24,
    PS2_F = 0x2b,
    PS2_G = 0x34,
    PS2_H = 0x33,
    PS2_I = 0x43,
    PS2_J = 0x3b,
    PS2_K = 0x42,
    PS2_L = 0x4b,
    PS2_M = 0x3a,
    PS2_N = 0x31,
    PS2_O = 0x44,
    PS2_P = 0x4d,
    PS2_Q = 0x15,
    PS2_R = 0x2d,
    PS2_S = 0x1b,
    PS2_T = 0x2c,
    PS2_U = 0x3c,
    PS2_V = 0x2a,
    PS2_W = 0x1d,
    PS2_X = 0x22,
    PS2_Y = 0x35,
    PS2_Z = 0x1a,
    PS2_0 = 0x45,
    PS2_1 = 0x16,
    PS2_2 = 0x1e,
    PS2_3 = 0x26,
    PS2_4 = 0x25,
    PS2_5 = 0x2e,
    PS2_6 = 0x36,
    PS2_7 = 0x3d,
    PS2_8 = 0x3e,
    PS2_9 = 0x46,
    PS2_LSHIFT = 0x12,
    PS2_LCTRL = 0x14,
    PS2_LALT = 0x11,
    PS2_RSHIFT = 0x59,
    PS2_BACKSPACE = 0x66,
    PS2_SPACE = 0x29,
    PS2_ESC = 0x76,
    PS2_ENTER = 0x5a;

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
    Pin * const _dat;
public:
    PS2Keyboard(Pin *dat);
    void timeTick() { _timeTicks++; }
    void isr();
    uint8_t get_scan_code() { return _buf.get(); }
};

#endif



