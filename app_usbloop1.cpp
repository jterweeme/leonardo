/*
reads from USB
prints to USB in opposite case
*/

#include "board.h"
#include "cdc.h"
#include "misc.h"

int main()
{
    Serial serial;
    serial.init();
    CDC usb;

    while (true)
    {
        uint8_t byte = usb.receive();

        if (byte != 255)
        {
            usb.sendByte(Utility::convertCase(byte));
            usb.flush();
        }
    }

    return 0;
}



