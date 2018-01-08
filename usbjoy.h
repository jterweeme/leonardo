#ifndef _USBJOY_H_
#define _USBJOY_H_
#include "busby.h"
#include "usbhid.h"

class USBJoy : public USB
{
private:
    Endpoint _inpoint;
    void procCtrlReq();
    uint16_t getDescriptor(uint16_t value, uint8_t wIndex, const void **descAddr);
public:
    USBJoy();
};

#endif



