#ifndef _USBJOY_H_
#define _USBJOY_H_
#include "busby.h"
#include "usbhid.h"

struct JoyReportData
{
    int8_t x;
    int8_t y;
    int8_t z;
    uint8_t button;
}
__attribute__ ((packed));

class USBJoy : public USB
{
private:
    Endpoint _inpoint;
    void procCtrlReq();
    uint16_t getDescriptor(uint16_t value, uint8_t wIndex, const void **descAddr);
public:
    void sendReport(JoyReportData &report);
    USBJoy();
};

#endif



