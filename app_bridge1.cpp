#include "busby.h"

ISR(USART1_RX_vect)
{
    uint8_t recv = UDR1;
}

int main()
{
    return 0;
}


