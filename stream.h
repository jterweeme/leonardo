#ifndef _STREAM_H_
#define _STREAM_H_
#include "cdc.h"

class ostream
{
public:
    virtual void write(char c) { }
    void write(const char *s) { while (*s) write(*s++); }
    virtual void writeString(const char *s) { while (*s) write(*s++); }
};

class USBStream : public ostream
{
    CDC _usb;
public:
    void write(char c) { _usb.sendByte(c); }
    void flush() { _usb.flush(); }
};

#endif



