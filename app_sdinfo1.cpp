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

static void printCSD(csd_t &csd, ostream &os)
{
    char buf[50];
#if 0
    for (uint8_t i = 0; i < 16; i++)
    {
        ::snprintf(buf, 50, "%u\r\n", csd.raw.byte[i]);
        os.writeString(buf);
        os.flush();
    }
#else
    uint8_t c_size_high = csd.v1.c_size_high;
    uint8_t c_size_mid = csd.v1.c_size_mid;
    uint8_t c_size_low = csd.v1.c_size_low;
    uint16_t c_size = c_size_high << 10 | c_size_mid << 2 | c_size_low;
    uint32_t size = c_size + 1;
    uint8_t c_size_mult_high = csd.v1.c_size_mult_high;
    uint8_t c_size_mult_low = csd.v1.c_size_mult_low;
    uint8_t c_size_mult = c_size_mult_high << 1 | c_size_mult_low;
    uint8_t multbits = c_size_mult + 2;
    uint32_t blocks = size << multbits;
    ::snprintf(buf, 50, "%lu x ", size);
    os.writeString(buf);
    os.flush();
    ::snprintf(buf, 50, "%u = ", 1<<multbits);
    os.writeString(buf);
    os.flush();
    ::snprintf(buf, 50, "%lu\r\n", blocks);
    os.writeString(buf);
    os.flush();
#endif
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
    csd_t csd;
    sd.readCSD(&csd);
    CDC cdc;
    USBStream cout(&cdc);

    while (true)
    {
        for (volatile uint32_t i = 0; i < 0xfffff; i++)
            ;

        printCSD(csd, cout);
    }

}


