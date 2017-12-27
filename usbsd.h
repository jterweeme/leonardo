#ifndef _USBSD_H_
#define _USBSD_H_
#include "busby.h"

class USBSD : public USB
{
private:
    volatile bool isMassStoreReset = false;
    Endpoint _inpoint;
    Endpoint _outpoint;
    uint16_t getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddr);
    void processCtrlReq();
    bool readInCmdBlock();
    bool decodeSCSICmd();
    bool scsiCmdInq();
    void retCmdStatus();
public:
    void msTask();
    void usbTask();
    USBSD();
    void gen();
    void com();
};

#endif



