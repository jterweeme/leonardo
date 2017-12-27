#ifndef _LEONARDO_H_
#define _LEONARDO_H_
#include <stdint.h>

static constexpr uint8_t
    portb_base = 0x23,
        pinb = portb_base,
        pinb0 = 0, pinb1 = 1, pinb2 = 2, pinb3 = 3, pinb4 = 4, pinb5 = 5, pinb6 = 6, pinb7 = 7,
        ddrb = portb_base + 1,
        ddrb0 = 0, ddrb1 = 1, ddrb2 = 2, ddrb3 = 3, ddrb4 = 4, ddrb5 = 5, ddrb6 = 6, ddrb7 = 7,
        portb = portb_base + 2,
        pb0 = 0, pb1 = 1, pb2 = 2, pb3 = 3, pb4 = 4, pb5 = 5, pb6 = 6, pb7 = 7,
    portc_base = 0x26,
        pinc = portc_base,
        pinc0 = 0, pinc1 = 1, pinc2 = 2, pinc3 = 3, pinc4 = 4, pinc5 = 5, pinc6 = 6, pinc7 = 7,
        ddrc = portc_base + 1,
        ddc0 = 0, ddc1 = 1, ddc2 = 2, ddc3 = 3,
        portc = portc_base + 2,
        pc6 = 6, pc7 = 7,
    portd_base = 0x29,
        pind = portd_base,
        ddrd = portd_base + 1,
        portd = portd_base + 2,
    porte_base = 0x2c,
        pine = porte_base,
        ddre = porte_base + 1,
        porte = porte_base + 2,
    portf_base = 0x2f,
        pinf = portf_base,
        ddrf = portf_base + 1,
        portf = portf_base + 2,
    timsk0 = 0x6e,
    toie0 = 0, ocie0a = 1, ocie0b = 2,
    timsk1 = 0x6f,
    toie1 = 0, ocie1a = 1, ocie1b = 2, ocie1c = 3, icie1 = 5,
    timsk3 = 0x71,
    toie3 = 0, ocie3a = 1, ocie3b = 2, ocie3c = 3, icie3 = 5,
    timsk4 = 0x72,
    toie4 = 2,
    ocie4b = 5,
    ocie4a = 6,
    ocie4d = 7,
    adcl = 0x78,
    adch = 0x79,
    adcsra = 0x7a,
        adps0 = 0, adps1 = 1, adps2 = 2, adie = 3, adif = 4, adate = 5, adsc = 6, aden = 7,
    adcsrb = 0x7b,
    admux = 0x7c,
        mux0 = 0, mux1 = 1, mux2 = 2, mux3 = 3, mux4 = 4, adlar = 5, refs0 = 6, refs1 = 7,
    tccr1a = 0x80,
    wgm10 = 0, wgm11 = 1, com1c0 = 2, com1c1 = 3, com1b0 = 4, com1b1 = 5, com1a0 = 6, com1a1 = 7,
    tccr1b = 0x81,
    cs10 = 0, cs11 = 1, cs12 = 2, wgm12 = 3, wgm13 = 4, ines1 = 6, icnc1 = 7,
    tccr1c = 0x82,
    foc1c = 5, foc1b = 6, foc1a = 7,
    tcnt1 = 0x84, tcnt1l = 0x84,
    ucsr1a = 0xc8,
        mpcm1 = 0, u2x1 = 1, upe1 = 2, dor1 = 3, fe1 = 4, udre1 = 5, txc1 = 6, rxc1 = 7,
    ucsr1b = 0xc9,
        txb81 = 0, rxb81 = 1, ucsz12 = 2, txen1 = 3,
        rxen1 = 4, udrie1 = 5, txcie1 = 6, rxcie1 = 7,
    ucsr1c = 0xca,
    ubrr1 = 0xcc,
    ubrr1l = 0xcc,
    ubrr1h = 0xcd,
    udr1 = 0xce,
    ss_port_base = portb_base,
    ss_ddr = ss_port_base + 1,
    ss_port = ss_port_base + 2,
    pss = pb0,
    sck_port_base = portb_base,
    sck_ddr = sck_port_base + 1,
    sck_port = sck_port_base + 2,
    psck = pb1,
    mosi_port_base = portb_base,
    mosi_ddr = mosi_port_base + 1,
    mosi_port = mosi_port_base + 2,
    pmosi = pb2,
    miso_port_base = portb_base,
    miso_ddr = miso_port_base + 1,
    miso_port = miso_port_base + 2,
    pmiso = pb3;

static volatile uint8_t
    *p_pinb = (volatile uint8_t * const)pinb,
    *p_ddrb = (volatile uint8_t * const)ddrb,
    *p_portb = (volatile uint8_t * const)portb,
    *p_pinc = (volatile uint8_t * const)pinc,
    *p_ddrc = (volatile uint8_t * const)ddrc,
    *p_portc = (volatile uint8_t * const)portc,
    *p_adcl = (volatile uint8_t * const)adcl,
    *p_adch = (volatile uint8_t * const)adch,
    *p_adcsra = (volatile uint8_t * const)adcsra,
    *p_adcsrb = (volatile uint8_t * const)adcsrb,
    *p_admux = (volatile uint8_t * const)admux,
    *p_ddr_ss = (volatile uint8_t * const)ss_ddr,
    *p_port_ss = (volatile uint8_t * const)ss_port,
    *p_ddr_sck = (volatile uint8_t * const)sck_ddr,
    *p_port_sck = (volatile uint8_t * const)sck_port,
    *p_ddr_mosi = (volatile uint8_t * const)mosi_ddr,
    *p_port_mosi = (volatile uint8_t * const)mosi_port,
    *p_ddr_miso = (volatile uint8_t * const)miso_ddr,
    *p_port_miso = (volatile uint8_t * const)miso_port,
    *p_tccr1a = (volatile uint8_t * const)tccr1a,
    *p_tccr1b = (volatile uint8_t * const)tccr1b,
    *p_tccr1c = (volatile uint8_t * const)tccr1c,
    *p_ucsr1a = (volatile uint8_t * const)ucsr1a,
    *p_ucsr1b = (volatile uint8_t * const)ucsr1b,
    *p_udr1 = (volatile uint8_t * const)udr1;

static volatile uint16_t
    *p_ubrr1 = (volatile uint16_t * const)ubrr1;

enum Bits { BIT0, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7 };

class Port
{
protected:
    volatile uint8_t * const pbase;
    volatile uint8_t * const pin;
public:
    volatile uint8_t * const direction;
    volatile uint8_t * const out;
    Port(uint8_t *base) : pbase(base), pin(base), direction(base + 1), out(base + 2) { }
    inline void setBit(Bits bit) { *out |= 1<<bit; }
    inline void clearBit(Bits bit) { *out &= ~(1<<bit); }
    inline void toggleBit(Bits bit) { *out ^= 1<<bit; }
    uint8_t read() { return *pin; }
    void write(uint8_t data) { *out = data; }
};

enum Direction { INPUT, OUTPUT };

struct Pin  // uses Port class
{
    Port &port;
    const Bits bit;
    Pin(Port &port, Bits bit) : port(port), bit(bit) { }
    inline void set() { port.setBit(bit); }
    inline void clear() { port.clearBit(bit); }
    inline void set(bool value) { return value ? set() : clear(); }
    inline void toggle() { port.toggleBit(bit); }
    void direction(Direction dir);
    bool read() { return port.read() & 1<<bit; }
};

struct Board
{
    Port
        portB { (uint8_t *)portb_base },
        portC { (uint8_t *)portc_base },
        portD { (uint8_t *)portd_base },
        portE { (uint8_t *)porte_base },
        portF { (uint8_t *)portf_base };

    Pin
        pin0 { portD, BIT2 },
        pin1 { portD, BIT3 },
        pin2 { portD, BIT1 },
        pin3 { portD, BIT0 },
        pin4 { portD, BIT4 },
        pin5 { portC, BIT6 },
        pin6 { portD, BIT7 },
        pin7 { portE, BIT6 },
        pin8 { portB, BIT4 },
        pin9 { portB, BIT5 },
        pin10 { portB, BIT6 },
        pin11 { portB, BIT7 },
        pin12 { portD, BIT6 },
        pin13 { portC, BIT7 },
        pinA0 { portF, BIT7 },
        pinA1 { portF, BIT6 },
        pinA2 { portF, BIT5 },
        pinA3 { portF, BIT4 },
        pinA4 { portF, BIT1 },
        pinA5 { portF, BIT0 };
};
#endif



