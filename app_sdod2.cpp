/*
makes a hexdump of an SD card, so doesn't use the
FAT code

D9: CS
*/

#include "zd2card.h"
#include <ctype.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "board.h"
#include "stream.h"

Sd2Card *g_sd;

ISR(TIMER0_OVF_vect)
{
    g_sd->tick();
}

void hexDump(uint8_t *point, ostream &os)
{
    for (uint16_t i = 0; i < 512; i += 16)
    {
        for (uint8_t j = 0; j < 16; j++)
        {
            char wbuf[10];  // write buffer
            snprintf(wbuf, 10, "%02x ", point[i + j]);
            os.write(wbuf);
        }

        for (uint8_t j = 0; j < 16; j++)
        {
            if (isprint(point[i + j]))
                os.write(point[i + j]);
            else
                os.write('.');
        }

        os.write("\r\n");
    }   
}

class Command
{
public:
    char _buf[50];
    uint8_t _pos = 0;
public:
    void push(uint8_t byte) { _buf[_pos++] = byte; }
    bool isCommand(); //{ return _buf[_pos - 1] == 'd' ? true : false; }
    uint32_t number();
    void reset() { _pos = 0; }
};

bool Command::isCommand()
{
    if (_pos <= 0)
        return false;

    return _buf[_pos - 1] == 'd' ? true : false;
}

static uint32_t myPow10(uint8_t exp)
{
    switch (exp)
    {
    case 0:
        return 1;
    case 1:
        return 10;
    case 2:
        return 100;
    case 3:
        return 1000;
    case 4:
        return 10000;
    case 5:
        return 100000;
    case 6:
        return 1000000;
    case 7:
        return 10000000;
    }

    return 1;
}

uint32_t Command::number()
{
    uint32_t sum = 0;
    
    for (int8_t i = _pos - 2, j = 0; i >= 0; i--, j++)
        sum += Utility::char2digit(_buf[i]) * myPow10(j);

    return sum;
}

int main()
{
    TCCR0B = 1<<CS02;
    TIMSK0 = 1<<TOIE0;
    sei();
    Board board;
    Sd2Card sd(&board.pin9);
    g_sd = &sd;
    sd.init(SPI_HALF_SPEED);
    uint8_t buf[512];
    CDC cdc;
    USBStream cout(&cdc);
    Command cmd;

    while (true)
    {
        uint8_t byte = cdc.receive();

        if (byte == 255)
            continue;

        cmd.push(byte);
        cout.write(cmd._buf[cmd._pos - 1]);
        cout.flush();

        if (cmd.isCommand())
        {
#if 0
            cout.writeString("\r\ndebug bericht\r\n");
            cout.flush();
            char buf2[50];
            snprintf(buf2, 50, "%u\r\n", cmd.number());
            cout.writeString(buf2);
            cout.flush();
#endif
            sd.readBlock(cmd.number(), buf);
            cmd.reset();
            cout.writeString("\r\n");
            hexDump(buf, cout);
            cout.flush();
        }
    }

    return 0;
}


