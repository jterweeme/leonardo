#include "busby2.h"
#include <avr/io.h>
#include <stddef.h>

USB *USB::instance;

USB::USB()
{
    instance = this;
}

uint8_t USB::Endpoint_GetCurrentEndpoint()
{
    return ((UENUM & ENDPOINT_EPNUM_MASK) | Endpoint_GetEndpointDirection());
}

uint8_t USB::Endpoint_GetEndpointDirection()
{
    return (UECFG0X & (1 << EPDIR)) ? ENDPOINT_DIR_IN : ENDPOINT_DIR_OUT;
}

void USB::Endpoint_ClearEndpoints()
{
    UEINT = 0;

    for (uint8_t EPNum = 0; EPNum < ENDPOINT_TOTAL_ENDPOINTS; EPNum++)
    {
        selectEndpoint(EPNum);
        UEIENX  = 0;
        UEINTX  = 0;
        UECFG1X = 0;
        UECONX &= ~(1<<EPEN);
    }
}

uint8_t USB::Endpoint_WaitUntilReady()
{
    uint8_t  TimeoutMSRem = 100; 
    uint16_t PreviousFrameNumber = UDFNUM;

    while (true)
    {
        if (Endpoint_GetEndpointDirection() == ENDPOINT_DIR_IN)
        {
            if (UEINTX & 1<<TXINI)
                return ENDPOINT_READYWAIT_NoError;
        }
        else
        {   
            if (UEINTX & 1<<RXOUTI)
                return ENDPOINT_READYWAIT_NoError;
        }

        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_READYWAIT_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_READYWAIT_BusSuspended;
        else if (UECONX & 1<<STALLRQ)
            return ENDPOINT_READYWAIT_EndpointStalled;
    
        uint16_t CurrentFrameNumber = UDFNUM;

        if (CurrentFrameNumber != PreviousFrameNumber)
        {
            PreviousFrameNumber = CurrentFrameNumber;
     
            if (!(TimeoutMSRem--))
                return ENDPOINT_READYWAIT_Timeout;
        }
    }
}

