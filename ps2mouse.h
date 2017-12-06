#ifndef _PS2MOUSE_H_
#define _PS2MOUSE_H_
#include <stdint.h>
#include <avr/io.h>

class PS2Mouse
{
public:
    void write(uint8_t dat);
    uint8_t read();
};

#endif



