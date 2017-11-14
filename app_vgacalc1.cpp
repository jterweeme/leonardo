#include "vga.h"
#include "calc.h"
#include "misc.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include "keyboard.h"
#include "busby.h"

static VGA vga;
static Calculator *g_calc;
static Sub sub;
//static Div div;
static Mul mul;

#if 1
static PS2Keyboard g_kb;

ISR(INT1_vect)
{
    TIMSK1 &= ~(1<<TOIE1);  // disable vsync interrupt
    //TIMSK2 &= ~(1<<TOIE2);  // disable hsync interrupt
    g_kb.isr();
    uint8_t scancode = g_kb.get_scan_code();

    if (scancode > 0)
    {
        switch (scancode)
        {
            case USKeyboard::N1:
                g_calc->push(1);
                break;
            case USKeyboard::N2:
                g_calc->push(2);
                break;
            case USKeyboard::N3:
                g_calc->push(3);
                break;
            case USKeyboard::N4:
                g_calc->push(4);
                break;
            case USKeyboard::N5:
                g_calc->push(5);
                break;
            case USKeyboard::N6:
                g_calc->push(6);
                break;
            case USKeyboard::N7:
                g_calc->push(7);
                break;
            case USKeyboard::N8:
                g_calc->push(8);
                break;
            case USKeyboard::N9:
                g_calc->push(9);
                break;
            case USKeyboard::N0:
                g_calc->push(0);
                break;
            case USKeyboard::MINUS:
                g_calc->op(&sub);
                break;
            case USKeyboard::STAR:
                g_calc->op(&mul);
                break;
            case USKeyboard::SLASH:
                //g_calc->op(&div);
                break;
            case USKeyboard::PLUS:
                g_calc->add();
                break;
            case USKeyboard::ENTER:
                g_calc->equals();
                break;
            case USKeyboard::ESC:
                g_calc->reset();
                break;
            }
    }

    TIMSK1 |= 1<<TOIE1;
    //TIMSK2 |= 1<<TOIE2;
}
#endif

class OutputVGA : public Output
{
    VGA *_vga;
    char _buf[20] = {0};
    uint8_t _pos = 0;
public:
    OutputVGA(VGA *v) : _vga(v) { }
    void redraw() const;
    inline void clear() { _buf[_pos = 0] = 0; redraw(); }
    void push(char c);
};

class DummyOutput : public Output
{
public:
    void clear() { }
    void push(char c) { }
};

void OutputVGA::redraw() const
{
    _vga->clear();
    _vga->gotoxy(20 - _pos, 0);

    for (uint8_t i = 0; i <= _pos; i++)
        _vga->write(_buf[i]);
}

void OutputVGA::push(char c)
{
    if (_pos >= 19)
        return;

    _buf[_pos++] = c;
    _buf[_pos] = 0;
    redraw();
}

inline char nibble(uint8_t n)
{
    return n <= 9 ? '0' + n : 'A' + n - 10;
}

int main()
{
    sei();
    vga.init();
    DummyOutput dum;
    OutputVGA cv(&vga);
    Calculator calc(&cv);
    g_calc = &calc;
    calc.reset();

    while (true)
        sleep_mode();

    return 0;
}


