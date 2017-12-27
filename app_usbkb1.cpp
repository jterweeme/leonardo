#include "usbkb.h"
#include "misc.h"
#include <string.h>

USB_KeyboardReport_Data_t report;

void hidTask(USBKB &kb)
{
    uint8_t i = 0;
    memset((void *)&report, 0, sizeof(report));

    if ((PINF & 1<<0) == 0)
        report.keyCode[i++] = HID_KEYBOARD_SC_A;

    if ((PINF & 1<<1) == 0)
        report.keyCode[i++] = HID_KEYBOARD_SC_B;

    if ((PINF & 1<<4) == 0)
        report.keyCode[i++] = HID_KEYBOARD_SC_C;

    if ((PINF & 1<<5) == 0)
        report.keyCode[i++] = HID_KEYBOARD_SC_D;

    if ((PINF & 1<<6) == 0)
        report.keyCode[i++] = HID_KEYBOARD_SC_E;

    if ((PINF & 1<<7) == 0)
        report.keyCode[i++] = HID_KEYBOARD_SC_F;

    if (i > 0)
        kb.sendReport(report);
}

int main()
{
    Serial serial;
    serial.init();
    DDRF &= ~(1<<0 | 1<<1 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    PORTF |= 1<<0 | 1<<1 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
    USBKB kb;
    
    while (true)
    {
        hidTask(kb);
        kb.usbTask();
    }

    return 0;
}


