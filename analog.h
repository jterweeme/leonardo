#ifndef _ANALOG_H_
#define _ANALOG_H_
#include <avr/io.h>

class Analog
{
public:
    static const uint8_t
        ADC0 = 0,
        ADC1 = 1<<MUX0,
        ADC2 = 1<<MUX1,
        ADC3 = 1<<MUX0 | 1<<MUX1,
        ADC4 = 1<<MUX2,
        ADC5 = 1<<MUX0 | 1<<MUX2;

    void init();
    uint16_t read(uint8_t input);
};

struct TSPoint
{
    uint16_t x = 0, y = 0, z = 0;
    TSPoint() { }
    TSPoint(uint16_t x1, uint16_t y1, uint16_t z1) : x(x1), y(y1), z(z1) { }
};

class TouchScreen
{
    Analog * const _adc;
    uint16_t _threshold = 200;
public:
    static const uint8_t SAMPLES = 2;
    TouchScreen(Analog *adc) : _adc(adc) { }
    TSPoint getPoint();
};

class DFKeypad
{
    Analog *_adc;
public:
    DFKeypad(Analog *adc) : _adc(adc) { }
    static const uint8_t UP = 1;
    static const uint8_t LEFT = 2;
    static const uint8_t RIGHT = 3;
    static const uint8_t DOWN = 4;
    static const uint8_t SELECT = 5;
    uint8_t read();
};

#endif



