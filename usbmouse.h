#ifndef _USBMOUSE_H_
#define _USBMOUSE_H_
#include "busby.h"

class USBMouse
{
private:
    Endpoint _inpoint;
    void procCtrlReq();
    uint16_t getDescriptor(uint16_t value, uint8_t wIndex, const void **descAddr);
public:
    USBMouse();
};

#endif



