#include "busby3.h"
#include <avr/pgmspace.h>

USB *USB::instance;

static inline uint8_t Endpoynt_BytesToEPSizeMask(const uint16_t bytes)
{
    uint8_t maskVal = 0;
    uint16_t checkBytes = 8;

    while (checkBytes < bytes)
    {
        maskVal++;
        checkBytes <<= 1;
    }

    return maskVal << EPSIZE0;
}

void USB::clearStatusStage()
{
    if (_ctrlReq.bmRequestType & REQDIR_DEVICETOHOST)
    {
        while ((*p_ueintx & 1<<rxouti) == 0)
            if (state == DEVICE_STATE_Unattached)
                return;

        *p_ueintx &= ~(1<<rxouti | 1<<fifocon); // clear out
    }
    else
    {
        while ((*p_ueintx & 1<<txini) == 0)
            if (state == DEVICE_STATE_Unattached)
                return; 

        *p_ueintx &= ~(1<<txini | 1<<fifocon);  // clear in
    }
}

uint8_t USB::readControlStreamLE(void * const buf, uint16_t Length)
{
    uint8_t* dataStream = (uint8_t*)buf;

    if (!Length)
        *p_ueintx &= ~(1<<rxouti | 1<<fifocon);

    while (Length)
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;

        if (*p_ueintx & 1<<rxstpi) // setup received?
            return ENDPOINT_RWCSTREAM_HostAborted;

        if (*p_ueintx & 1<<rxouti)  // out received?
        {
            while (Length && Endpoynt_BytesInEndpoint())
            {
                *dataStream = read8();
                dataStream++;
                Length--;
            }

            *p_ueintx &= ~(1<<rxouti | 1<<fifocon); // clear out
        }
    }

    while ((*p_ueintx & 1<<txini) == 0)
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}


uint8_t USB::write_Control_Stream_LE(const void * const buf, uint16_t Length)
{
    uint8_t* DataStream     = ((uint8_t*)buf);
    bool     LastPacketFull = false;

    if (Length > _ctrlReq.wLength)
        Length = _ctrlReq.wLength;
    else if (!(Length))
        *p_ueintx &= ~(1<<txini | 1<<fifocon);  // endpoint clear in

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;

        if (*p_ueintx & 1<<rxstpi)  // setup received?
            return ENDPOINT_RWCSTREAM_HostAborted;

        if (*p_ueintx & 1<<rxouti)  // out received?
            break;

        if (*p_ueintx & 1<<txini)
        {
            uint16_t bytesInEp = Endpoynt_BytesInEndpoint();

            while (Length && bytesInEp < 8) // control EP size
            {
                write8(*DataStream);
                DataStream++;
                Length--;
                bytesInEp++;
            }

            LastPacketFull = bytesInEp == 8;    // control EP size
            *p_ueintx &= ~(1<<txini | 1<<fifocon);  // endpoint clear in
        }
    }

    while ((Endpoynt_IsOUTReceived()) == 0)
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;

        if (*p_ueintx & 1<<rxstpi)
            return ENDPOINT_RWCSTREAM_HostAborted;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

void USB::Device_ClearSetFeature()
{
    switch (_ctrlReq.bmRequestType & CONTROL_REQTYPE_RECIPIENT)
    {
    case REQREC_DEVICE:
        if ((uint8_t)_ctrlReq.wValue == FEATURE_SEL_DeviceRemoteWakeup)
            USB_Device_RemoteWakeupEnabled = (_ctrlReq.bRequest == REQ_SetFeature);
        else
            return;

        break;
    case REQREC_ENDPOINT:
        if ((uint8_t)_ctrlReq.wValue == FEATURE_SEL_EndpointHalt)
        {
            uint8_t EndpointIndex = ((uint8_t)_ctrlReq.wIndex & ENDPOINT_EPNUM_MASK);

            if (EndpointIndex == 0 || EndpointIndex >= ENDPOINT_TOTAL_ENDPOINTS)
                return;

            selectEndpoint(EndpointIndex);

            if (UECONX & 1<<EPEN)
            {
                if (_ctrlReq.bRequest == REQ_SetFeature)
                {
                    UECONX |= 1<<STALLRQ;
                }
                else
                {
                    *p_ueconx |= 1<<STALLRQC;
                    *p_uerst = 1<<(EndpointIndex & ENDPOINT_EPNUM_MASK);
                    *p_uerst = 0;
                    *p_ueconx |= 1<<rstdt;
                }
            }
        }

        break;
    default:
        return;
    }

    selectEndpoint(0);
    *p_ueintx &= ~(1<<rxstpi);
    clearStatusStage();
}


uint8_t USB::write_Control_PStream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = ((uint8_t*)Buffer);
    bool     LastPacketFull = false;

    if (Length > _ctrlReq.wLength)
        Length = _ctrlReq.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;

        if (UEINTX & 1<<RXSTPI) // setup received?
            return ENDPOINT_RWCSTREAM_HostAborted;

        if (Endpoynt_IsOUTReceived())
            break;

        if (UEINTX & 1<<TXINI)
        {
            uint16_t BytesInEndpoint = Endpoynt_BytesInEndpoint();

            while (Length && (BytesInEndpoint < 8)) // control EP size
            {
                write8(pgm_read_byte(DataStream));
                DataStream += 1;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = (BytesInEndpoint == 8);    // control EP size
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
        }
    }

    while ((Endpoynt_IsOUTReceived()) == 0)
    {
        if (state == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;

        if (state == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;

        if (UEINTX & 1<<RXSTPI)
            return ENDPOINT_RWCSTREAM_HostAborted;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

void USB::gen()
{
    if (*p_udint & 1<<sofi && *p_udien & 1<<sofe)
        *p_udint &= ~(1<<sofi);

    if (*p_usbint & 1<<vbusti && *p_usbcon & 1<<vbuste)
    {
        *p_usbint &= ~(1<<vbusti);

        if (*p_usbsta & 1<<vbus)
        {
            *p_pllcsr = 1<<pindiv;
            *p_pllcsr = 1<<pindiv | 1<<plle;

            while ((*p_pllcsr & 1<<plock) == 0)
                ;

            state = DEVICE_STATE_Powered;
            connect();
        }
        else
        {
            PLLCSR = 0;
            state = DEVICE_STATE_Unattached;
            //EVENT_USB_Device_Disconnect();
        }
    }

    if (*p_udint & 1<<suspi && *p_udien & 1<<suspe)
    {
        *p_udien &= ~(1<<suspe);
        *p_udien |= 1<<wakeupe;
        *p_usbcon |= 1<<frzclk;
        *p_pllcsr = 0;
        state = DEVICE_STATE_Suspended;
    }

    if (*p_udint & 1<<wakeupi && *p_udien & 1<<wakeupe)
    {
        *p_pllcsr = 1<<pindiv;
        *p_pllcsr = 1<<pindiv | 1<<plle;

        while ((*p_pllcsr & 1<<plock) == 0)
            ;

        *p_usbcon &= ~(1<<frzclk);
        *p_udint &= ~(1<<wakeupi);
        *p_udien &= ~(1<<wakeupe);
        *p_udien |= 1<<suspe;

        if (USB_Device_ConfigurationNumber)
            state = DEVICE_STATE_Configured;
        else
            state = *p_udaddr & 1<<adden ? DEVICE_STATE_Addressed : DEVICE_STATE_Powered;

    }

    if (*p_udint & 1<<eorsti && *p_udien & 1<<eorste)
    {
        *p_udint &= ~(1<<eorsti);      // clear INT EORSTI
        state = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        *p_udint &= ~(1<<suspi);       // clear INT SUSPI
        *p_udien &= ~(1<<suspe);       // disable INT SUSPE
        *p_udien |= 1<<wakeupe;
        configureEndpoint(0, 0, 8, 1);
        *p_ueienx |= 1<<rxstpe;
    }
}

void USB::com()
{
    uint8_t PrevSelectedEndpoint = getCurrentEndpoint();
    selectEndpoint(0);
    *p_ueienx &= ~(1<<rxstpe);
    sei();
    procCtrlReq();
    selectEndpoint(0);
    *p_ueienx |= 1<<rxstpe;
    selectEndpoint(PrevSelectedEndpoint);
}

bool USB::Endpoynt_ConfigureEndpoint_Prv(const uint8_t Number,
                                    const uint8_t UECFG0XData,
                                    const uint8_t UECFG1XData)
{
    for (uint8_t EPNum = Number; EPNum < ENDPOINT_TOTAL_ENDPOINTS; EPNum++)
    {
        uint8_t UECFG0XTemp;
        uint8_t UECFG1XTemp;
        uint8_t UEIENXTemp;

        selectEndpoint(EPNum);

        if (EPNum == Number)
        {
            UECFG0XTemp = UECFG0XData;
            UECFG1XTemp = UECFG1XData;
            UEIENXTemp  = 0;
        }
        else
        {
            UECFG0XTemp = UECFG0X;
            UECFG1XTemp = UECFG1X;
            UEIENXTemp  = UEIENX;
        }

        if ((UECFG1XTemp & 1<<ALLOC) == 0)
            continue;

        *p_ueconx &= ~(1<<epen);       // disable endpoint
        UECFG1X &= ~(1<<alloc);
        *p_ueconx |= 1<<epen;          // enable endpoint
        UECFG0X = UECFG0XTemp;
        UECFG1X = UECFG1XTemp;
        UEIENX  = UEIENXTemp;

        if ((UESTA0X & 1<<CFGOK) == 0)
          return false;
    }

    selectEndpoint(Number);
    return true;
}


bool USB::configureEndpoint(const uint8_t Address,
                     const uint8_t Type, const uint16_t Size, const uint8_t Banks)
{
    uint8_t Number = (Address & ENDPOINT_EPNUM_MASK);

    if (Number >= ENDPOINT_TOTAL_ENDPOINTS)
        return false;

    return Endpoynt_ConfigureEndpoint_Prv(Number,
                   ((Type << EPTYPE0) | ((Address & ENDPOINT_DIR_IN) ? (1 << EPDIR) : 0)),
         (1<<ALLOC | ((Banks > 1) ? (1 << EPBK0) : 0) | Endpoynt_BytesToEPSizeMask(Size)));
}

USB::USB() : _control(ENDPOINT_CONTROLEP, 8, EP_TYPE_CONTROL, 1)
{
    instance = this;
}

ISR(USB_COM_vect)
{
    USB::instance->com();
}

ISR(USB_GEN_vect)
{
    USB::instance->gen();
}



