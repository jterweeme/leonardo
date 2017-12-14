#define F_CPU 16000000

#include "board.h"
#include "misc.h"
#include <stdarg.h>

static inline void _delay_loop_1(uint8_t __count) __attribute__((always_inline));
static inline void _delay_loop_2(uint16_t __count) __attribute__((always_inline));

void _delay_loop_1(uint8_t __count)
{
    __asm__ volatile (
        "1: dec %0" "\n\t"
        "brne 1b"
        : "=r" (__count)
        : "0" (__count)
    );
}

void _delay_loop_2(uint16_t __count)
{
    __asm__ volatile (
        "1: sbiw %0,1" "\n\t"
        "brne 1b"
        : "=w" (__count)
        : "0" (__count)
    );
}

static inline void _delay_us(double __us) __attribute__((always_inline));
static inline void _delay_ms(double __ms) __attribute__((always_inline));

void _delay_ms(double __ms)
{
    double __tmp ;
    uint16_t __ticks;
    __tmp = ((F_CPU) / 4e3) * __ms;

    if (__tmp < 1.0)
    {
        __ticks = 1;
    }
    else if (__tmp > 65535)
    {
        __ticks = (uint16_t) (__ms * 10.0);

        while(__ticks)
        {
            _delay_loop_2(((F_CPU) / 4e3) / 10);
            __ticks --;
        }

        return;
    }
    else
    {
        __ticks = (uint16_t)__tmp;
    }
    _delay_loop_2(__ticks);
}

void _delay_us(double __us)
{
    double __tmp ;
#if __HAS_DELAY_CYCLES && defined(__OPTIMIZE__) && \
  !defined(__DELAY_BACKWARD_COMPATIBLE__) &&       \
  __STDC_HOSTED__
    uint32_t __ticks_dc;
    extern void __builtin_avr_delay_cycles(unsigned long);
    __tmp = ((F_CPU) / 1e6) * __us;

    #if defined(__DELAY_ROUND_DOWN__)
        __ticks_dc = (uint32_t)fabs(__tmp);

    #elif defined(__DELAY_ROUND_CLOSEST__)
        __ticks_dc = (uint32_t)(fabs(__tmp)+0.5);

    #else
        __ticks_dc = (uint32_t)(ceil(fabs(__tmp)));
    #endif

    __builtin_avr_delay_cycles(__ticks_dc);

#else
    uint8_t __ticks;
    double __tmp2 ;
    __tmp = ((F_CPU) / 3e6) * __us;
    __tmp2 = ((F_CPU) / 4e6) * __us;
    if (__tmp < 1.0)
        __ticks = 1;
    else if (__tmp2 > 65535)
    {
        _delay_ms(__us / 1000.0);
    }
    else if (__tmp > 255)
    {
        uint16_t __ticks=(uint16_t)__tmp2;
        _delay_loop_2(__ticks);
        return;
    }
    else
        __ticks = (uint8_t)__tmp;
    _delay_loop_1(__ticks);
#endif
}

char *Utility::itoa(int num, char *str, int base)
{
    int i = 0;
    bool isNegative = false;
 
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }
 
    if (num < 0 && base == 10)
    {
        isNegative = true;
        num = -num;
    }
 
    while (num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }
 
    if (isNegative)
        str[i++] = '-';
 
    str[i] = '\0';
    reverse(str, i);
    return str;
}

void CircBuf::push(uint8_t v)
{
    uint8_t i = head + 1;

    if (i >= BUFFER_SIZE)
        i = 0;

    if (i != tail)
    {
        _buf[i] = v;
        head = i;
    }
}

uint8_t CircBuf::get(uint8_t pos)
{
    if (empty())
        return 0;

    tail = pos;
    return _buf[pos];
}

uint8_t CircBuf::get()
{
    uint8_t i = tail;

    if (++i >= BUFFER_SIZE)
        i = 0;

    return get(i);
}

Serial *Serial::instance;

void Serial::init() const
{
    *p_ubrr1 = 51;    // 9600baud @16MHz
    *p_ucsr1b = 1<<txen1;
}

void Serial::write(char c) const
{
    while (!(*p_ucsr1a & 1<<udre1))
        ;

    *p_udr1 = c;
}

uint8_t Serial::readByte() const
{
    while (!(*p_ucsr1a & 1<<rxc1))
        ;

    return *p_udr1;
}

#if 0
void Pin::direction(Direction dir)
{
    switch (dir)
    {
    case INPUT:
        *port.direction &= ~(1<<bit);
        break;
    case OUTPUT:
        *port.direction |= (1<<bit);
        break;
    }
}
#endif

void Utility::reverse(char str[], int length)
{
    int start = 0;
    int end = length - 1;
    
    while (start < end)
    {
        swap(*(str + start), *(str + end));
        start++;
        end--;
    }
}

int Utility::strcmp(const char *a, const char *b)
{
    for (; *a == *b; a++, b++)
        if (*a == '\0')
            return 0;

    return (*a - *b);
}

size_t Utility::strlen(const char *s)
{
    const char *t;
    for (t = s; *t; ++t) { }
    return (t - s);
}





