#ifndef _USBMOUSE_H_
#define _USBMOUSE_H_
#include "busby.h"

struct MouseReportData
{
    uint8_t button = 0;
    int8_t x = 1;
    int8_t y = 0;
} __attribute__ ((packed));

class USBMouse : public USB
{
private:
    Endpoint _inpoint;
    void procCtrlReq();
    uint16_t getDescriptor(uint16_t value, uint8_t wIndex, const void **descAddr);
public:
    void sendReport(MouseReportData &mouseReport);
    USBMouse();
};

#endif



