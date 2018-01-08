#include "analog.h"
#include "misc.h"
#include "stream.h"
#include <stdio.h>

static constexpr uint8_t SAMPLES = 2;

static void getPoint(Analog &analog, ostream &os)
{
    int samples[SAMPLES];
    uint8_t valid = 1;
    DDRF &= ~(1<<4);    //A3
    DDRB &= ~(1<<5);    //D9
    DDRB |= 1<<4;       //D8
    DDRF |= 1<<5;       //A2
    PORTB |= 1<<4;      //D8
    PORTF &= ~(1<<5);   //A2

    for (uint8_t i = 0; i < SAMPLES; i++)
        samples[i] = analog.read(Analog::ADC4); //A3

    if (samples[0] - samples[1] < -4 || samples[0] - samples[1] > 4)
        valid = 0;
    else
        samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples

    //uint16_t x = 1023 - samples[2/2];
    uint16_t x = samples[2/2] % 1024;
    x -= 200;
    x /= 2.9;
    DDRB &= ~(1<<4);    //D8
    DDRF &= ~(1<<5);    //A2
    DDRF |= 1<<4;       //A3
    DDRB |= 1<<5;       //D9
    PORTB &= ~(1<<5);   //D9
    PORTF |= 1<<4;      //A3

    for (uint8_t i = 0; i < SAMPLES; i++)
        samples[i] = analog.read(Analog::ADC5); //A2

    if (samples[0] - samples[1] < -4 || samples[0] - samples[1] > 4)
        valid = 0;
    else
        samples[1] = (samples[0] + samples[1]) >> 1;

    uint16_t y = 1023 - samples[SAMPLES/2];
    y -= 90;
    y /= 2.5;
    DDRB |= 1<<4;       //xp    //D8
    DDRF &= ~(1<<4);    //yp    //A3
    PORTB &= ~(1<<4);   //xp    //D8
    PORTB |= 1<<5;      //ym    //D9
    uint16_t z1 = analog.read(Analog::ADC5);    //A2
    uint16_t z2 = analog.read(Analog::ADC4);    //A3
    uint16_t z = 1023 - (z2 - z1);

    if (!valid)
        z = 0;

    char buf[100];
    snprintf(buf, 100, "%u %u %u\r\n", x, y, z);
    os.writeString(buf);
    os.flush();
}

int main()
{
    CDC cdc;
    USBStream cout(&cdc);
    Analog a;
    a.init();

    while (true)
        getPoint(a, cout);

    return 0;
}



