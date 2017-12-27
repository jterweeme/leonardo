#ifndef _STREAM_H_
#define _STREAM_H_
#include "cdc.h"

class ostream
{
public:
    virtual void write(char c) { }
    void write(const char *s) { while (*s) write(*s++); }
    virtual void writeString(const char *s) { while (*s) write(*s++); }
    virtual void flush() { }
};

class USBStream : public ostream
{
    CDC * const _cdc;
public:
    USBStream(CDC *cdc) : _cdc(cdc) { }
    void write(char c) { _cdc->sendByte(c); }
    void flush() { _cdc->flush(); }
};

#endif



