#include "analog.h"
#include "cdc.h"
#include <stdio.h>
#include "stream.h"

int main()
{
    CDC cdc;
    USBStream s(&cdc);
    Analog analog;
    analog.init();

    while (true)
    {
        uint16_t value = analog.read(Analog::ADC0);
        char buf[50];
        snprintf(buf, 50, "%u\r\n", value);
        s.writeString(buf);
        s.flush();
    }

    return 0;
}


