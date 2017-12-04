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

UartBase::UartBase(uint16_t *brr, uint8_t *udr, uint8_t *ucsra, uint8_t *ucsrb)
  :
    brr(brr),
    udr(udr),
    ucsra(ucsra),
    ucsrb(ucsrb)
{
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


void Terminal::printf(const char *format, ...)
{
    va_list argp;
    va_start(argp, format);

    for (const char *p = format; *p != '\0'; p++)
    {
        if (*p != '%')
        {
            myPutc(*p);
            continue;
        }

        switch (*++p)
        {
        case 'u':
            {
                unsigned u = va_arg(argp, unsigned);
                char s[40] = {0};
                puts(Utility::itoa(u, s, 10));
            }
            
            break;
        case 's':
            {
                char *s = va_arg(argp, char *);
                puts(s);
            }

            break;
        case 'x':
            {
                unsigned u = va_arg(argp, unsigned);
                const uint8_t foo = u & 0x0f;
                const uint8_t bar = u >> 4;
                char high = bar < 10 ? (char)(bar + 48) : (char)(bar + 55);
                char low = foo < 10 ? (char)(foo + 48) : (char)(foo + 55);
                myPutc(high);
                myPutc(low);
            }
            break;
        }
    }
    
    va_end(argp);
}

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

string::string(const char *s)
{
    _capacity = Utility::strlen(s) + 1;
    this->s = new char[_capacity];
    Utility::strcpy(this->s, s);
}

void string::append(const char *s)
{
    append(string(s));
}

void string::append(const string &s)
{
    Utility::strcpy(this->s + size(), s.c_str());
}

AnalogBase::AnalogBase(uint8_t *base)
  :
    base(base),
    adcl(base),
    adch(base + 1),
    adcsra(base + 2),
    adcsrb(base + 3),
    admux(base + 4)
{
}

Analog::Analog() : AnalogBase((uint8_t *)0x78)
{
    *admux = 0x00 | (1<<BREFS0);
    *adcsra = (1<<BADPS1) | (1<<BADPS0) | (1<<BADEN) | (1<<BADATE);
    *adcsrb &= (1<<BADTS2) | (1<<BADTS1) | (1<<BADTS0);
    *adcsra |= (1<<BADSC);
}

uint8_t SPIBase::transfer(uint8_t data)
{
    *spdr = data;
    while (!(*spsr & spsr_bits::spif)) { }
    return *spdr;
}

DS1302::DS1302(Pin *clk, Pin *dat, Pin *rst) : clk(clk), dat(dat), rst(rst)
{
    clk->direction(OUTPUT);
    dat->direction(OUTPUT);
    rst->direction(OUTPUT);
    write(ENABLE, 0);
    write(TRICKLE, 0);
}

void DS1302::write(int address, uint8_t data)
{
    address &= ~(1<<0);
    start();
    toggleWrite(address, false);
    toggleWrite(data, false);
    stop();
}

void LCD::home()
{
    lcd_write_instruction_4d(CURSOR_HOME);
    _delay_ms(2);
}

bitset<8> DS1302::toggleReadBitset()
{
    bitset<8> dataBitset;
    
    for (uint8_t i = 0; i <= 7; i++)
    {
        clk->set();
        _delay_us(1);
        clk->clear();
        _delay_us(1);
        dataBitset.set(i, dat->read());
        
    }
    return dataBitset;

}

void DS1302::toggleWrite(uint8_t data, uint8_t release)
{
    for (int i = 0; i <= 7; i++)
    {
        bitset<8> dataBitset(data);
        dat->set(dataBitset.test(i));
        _delay_us(1);
        clk->set();
        _delay_us(1);

        if (release && i == 7)
        {
            dat->direction(INPUT);
        }
            else
        {
            clk->clear();
            _delay_us(1);
        }
    }
}

void DS1302::update()
{
    clockBurstRead((uint8_t *)&regs);
}

void DS1302::clockBurstRead(uint8_t *p)
{
    start();
    toggleWrite(CLOCK_BURST_READ, true);
    
    for (int i = 0; i < 8; i++)
        *p++ = toggleRead();

    stop();
}

void DS1302::clockBurstWrite(uint8_t *p)
{
    start();
    toggleWrite(CLOCK_BURST_WRITE, false);

    for (int i = 0; i < 8; i++)
        toggleWrite(*p++, false);

    stop();
}

void DS1302::start()
{
    rst->clear();
    rst->direction(OUTPUT);
    clk->direction(OUTPUT);
    dat->direction(OUTPUT);
    rst->set();
    _delay_us(4);
}

void DS1302::stop()
{
    rst->clear();
    _delay_us(4);
}

LCD::LCD(Pin *rs, Pin *e, Pin *d4, Pin *d5, Pin *d6, Pin *d7)
  :
    rs(rs), e(e), d4(d4), d5(d5), d6(d6), d7(d7)
{
    rs->direction(OUTPUT);
    e->direction(OUTPUT);
    d4->direction(OUTPUT);
    d5->direction(OUTPUT);
    d6->direction(OUTPUT);
    d7->direction(OUTPUT);
    _delay_ms(100);
    rs->clear();
    e->clear();
    lcd_write_4(RESET);
    _delay_ms(10);
    lcd_write_4(RESET);
    _delay_us(200);
    lcd_write_4(RESET);
    _delay_us(200);
    lcd_write_4(FOURBIT);
    _delay_us(80);
    lcd_write_instruction_4d(FOURBIT);
    _delay_us(80);
    lcd_write_instruction_4d(OFF);
    _delay_us(80);
    lcd_write_instruction_4d(CLEAR);
    _delay_ms(4);
    lcd_write_instruction_4d(ENTRYMODE);
    _delay_us(80);
    lcd_write_instruction_4d(ON);
    _delay_us(80);
}

void LCD::lcd_write_4(uint8_t theByte)
{
    d7->set(theByte & 1<<7);
    d6->set(theByte & 1<<6);
    d5->set(theByte & 1<<5);
    d4->set(theByte & 1<<4);
    e->set();
    _delay_us(1);
    e->clear();
    _delay_us(1);
}

void LCD::myPutc(char c)
{
    if (c == '\r')
        return home();

    rs->set();
    e->clear();
    lcd_write_4(c);
    lcd_write_4(c << 4);
}

void LCD::puts(const char *s)
{
    while (*s)
    {
        myPutc(*s++);
        _delay_us(80);
    }
}

void LCD::lcd_write_instruction_4d(uint8_t theInstruction)
{
    rs->clear();
    e->clear();
    lcd_write_4(theInstruction);
    lcd_write_4(theInstruction << 4);

}

uint8_t DFKeyPad::poll()
{
    unsigned x = adc.read();
    if (x < 100) return RIGHT;
    if (x < 250) return UP;
    if (x < 400) return DOWN;
    if (x < 550) return LEFT;
    if (x < 800) return SELECT;
    return 0;
}



