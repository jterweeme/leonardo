/*
Calculator die gebruik maakt van USB CDC

werkt
*/

#include "calc.h"
#include <avr/interrupt.h>
#include "misc.h"
#include "stream.h"

Calculator *g_calc;
Sub sub;
Div g_div;
Mul mul;

void command(char c)
{
    switch (c)
    {
        case '1':
            g_calc->push(1);
            break;
        case '2':
            g_calc->push(2);
            break;
        case '3':
            g_calc->push(3);
            break;
        case '4':
            g_calc->push(4);
            break;
        case '5':
            g_calc->push(5);
            break;
        case '6':
            g_calc->push(6);
            break;
        case '7':
            g_calc->push(7);
            break;
        case '8':
            g_calc->push(8);
            break;
        case '9':
            g_calc->push(9);
            break;
        case '0':
            g_calc->push(0);
            break;
        case '-':
            g_calc->op(&sub);
            break;
        case '*':
            g_calc->op(&mul);
            break;
        case '/':
            g_calc->op(&g_div);
            break;
        case '=':
            g_calc->equals();
            break;
        case '+':
            g_calc->add();
            break;
        case 'c':
            g_calc->reset();
            break;
    }
}

class OutputTerminal : public Output
{
private:
    ostream * const _os;
    char _buf[20] = {0};
    uint8_t _pos = 0;
public:
    OutputTerminal(ostream *os) : _os(os) { }
    void redraw();
    void clear();
    void push(char c);
};

void OutputTerminal::redraw()
{
    _os->write('\r');

    for (uint8_t i = 0; i <= 20 - _pos; i++)
        _os->write(' ');

    for (uint8_t i = 0; i <= _pos; i++)
        _os->write(_buf[i]);

    _os->flush();
}

void OutputTerminal::push(char c)
{
    if (_pos >= 19)
        return;

    _buf[_pos++] = c;
    _buf[_pos] = 0;     // null terminate
    redraw();
}

void OutputTerminal::clear()
{
    _buf[_pos = 0] = 0;
    redraw();
}

int main()
{
    CDC cdc;
    USBStream os(&cdc);
    OutputTerminal ot(&os);
    sei();
    Calculator calc(&ot);
    g_calc = &calc;
    calc.reset();

    while (true)
    {
        uint8_t c = cdc.receive();

        if (c != 255)
        {
            command(c);
        }
    }

    return 0;
}


