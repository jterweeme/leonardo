#ifndef _MISC_H_
#define _MISC_H_
#include <stdlib.h>
#include "leonardo.h"

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;

class Utility
{
public:
    static inline void strcpy(char *d, const char *s) { while ((*d++ = *s++)); }
    static inline void delay(const uint32_t x) { for (volatile uint32_t i = 0; i < x; i++); }
    template <class T> static void swap(T& a, T& b) { T c(a); a = b; b = c; }
    static void reverse(char str[], int length);
    static int strcmp(const char *a, const char *b);
    static size_t strlen(const char *s);
    static char *itoa(int num, char *str, int base);
    static void *malloc(size_t size) { return ::malloc(size); }
};

class Serial
{
public:
    static Serial *instance;
    Serial() { instance = this; }
    void init() const;
    inline void enableRead() const { *p_ucsr1b |= 1<<rxen1; }
    inline void enableReadInterrupt() const { *p_ucsr1b |= 1<<rxcie1; }
    uint8_t readByte() const;
    void write(char c) const;
    inline void write(const char *s) const { while (*s) write(*s++); }
};

class Terminal
{
public:
    virtual void myPutc(char c) { }
    virtual void puts(const char *s) { while (*s) myPutc(*s++); }
    virtual void printf(const char *format, ...);
    virtual uint8_t readByte() { return 0; }
    void operator<< (const char *s) { puts(s); }
};

class CircBuf
{
    static const uint8_t BUFFER_SIZE = 5;
    volatile uint8_t _buf[BUFFER_SIZE];
public:
    volatile uint8_t head = 0, tail = 0;
    void push(uint8_t v);
    bool empty() const { return head == tail; }
    uint8_t get(uint8_t pos);
    uint8_t get();
};



#endif


