#include "busby.h"
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

USB *USB::instance;

uint8_t USB::getEndpointDirection()
{
    return UECFG0X & 1<<EPDIR ? ENDPOINT_DIR_IN : ENDPOINT_DIR_OUT;
}

uint32_t USB::read32()
{
    union
    {
        uint32_t Value;
        uint8_t  Bytes[4];
    } Data;

    Data.Bytes[0] = UEDATX;
    Data.Bytes[1] = UEDATX;
    Data.Bytes[2] = UEDATX;
    Data.Bytes[3] = UEDATX;
    return Data.Value;
}

void USB::write32(uint32_t Data)
{
    UEDATX = Data &  0xFF;
    UEDATX = Data >> 8;
    UEDATX = Data >> 16;
    UEDATX = Data >> 24;
}

void USB::Device_GetSerialString(uint16_t *UnicodeString)
{
    uint_reg_t CurrentGlobalInt = SREG;
    cli();
    uint8_t SigReadAddress = INTERNAL_SERIAL_START_ADDRESS;

    for (uint8_t SerialCharNum = 0;
        SerialCharNum < INTERNAL_SERIAL_LENGTH_BITS / 4; SerialCharNum++)
    {
        uint8_t SerialByte = boot_signature_byte_get(SigReadAddress);

        if (SerialCharNum & 0x01)
        {
            SerialByte >>= 4;
            SigReadAddress++;
        }

        SerialByte &= 0x0F;

        UnicodeString[SerialCharNum] = SerialByte >= 10 ?
            ('A' - 10) + SerialByte : '0' + SerialByte;
    }

    GCC_MEMORY_BARRIER();
    SREG = CurrentGlobalInt;
    GCC_MEMORY_BARRIER();
}

uint8_t USB::Endpoint_WaitUntilReady()
{
    uint16_t TimeoutMSRem = USB_STREAM_TIMEOUT_MS;
    uint16_t PreviousFrameNumber = UDFNUM;

    while (true)
    {
        if (getEndpointDirection() == ENDPOINT_DIR_IN)
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

void USB::Endpoint_ClearStatusStage()
{
    if (USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST)
    {
        while ((UEINTX & 1<<RXOUTI) == 0)
            if (state == DEVICE_STATE_Unattached)
                return;

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }
    else
    {
        while ((UEINTX & 1<<TXINI) == 0)
            if (state == DEVICE_STATE_Unattached)
                return;

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
    }
}

bool USB::configureEndpoint(uint8_t addr, uint8_t Type, uint16_t Size, uint8_t banks)
{
    uint8_t Number = addr & ENDPOINT_EPNUM_MASK;

    if (Number >= ENDPOINT_TOTAL_ENDPOINTS)
        return false;

    uint8_t cfg0 = Type << EPTYPE0 | (addr & ENDPOINT_DIR_IN ? 1<<EPDIR : 0);
    uint8_t cfg1 = 1<<ALLOC | (banks > 1 ? 1<<EPBK0 : 0) | Endpoint_BytesToEPSizeMask(Size);

    for (uint8_t EPNum = Number; EPNum < ENDPOINT_TOTAL_ENDPOINTS; EPNum++)
    {
        uint8_t UECFG0XTemp, UECFG1XTemp, UEIENXTemp;
        selectEndpoint(EPNum);

        if (EPNum == Number)
        {
            UECFG0XTemp = cfg0;
            UECFG1XTemp = cfg1;
            UEIENXTemp  = 0;
        }
        else
        {
            UECFG0XTemp = UECFG0X;
            UECFG1XTemp = UECFG1X;
            UEIENXTemp  = UEIENX;
        }

        if (!(UECFG1XTemp & 1<<ALLOC))
          continue;

        UECONX &= ~(1<<EPEN);
        UECFG1X &= ~(1<<ALLOC);
        UECONX |= 1<<EPEN;
        UECFG0X = UECFG0XTemp;
        UECFG1X = UECFG1XTemp;
        UEIENX = UEIENXTemp;

       if (!(UESTA0X & 1<<CFGOK))
          return false;
    }

    selectEndpoint(Number);
    return true;
}

USB::USB() :
    _control(ENDPOINT_CONTROLEP, 8, EP_TYPE_CONTROL, 1)
{
    instance = this;
}

void USB::Device_ClearSetFeature()
{
    switch (USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_RECIPIENT)
    {
    case REQREC_DEVICE:
        if ((uint8_t)USB_ControlRequest.wValue == FEATURE_SEL_DeviceRemoteWakeup)
            USB_Device_RemoteWakeupEnabled = USB_ControlRequest.bRequest == REQ_SetFeature;
        else
            return;

        break;
    case REQREC_ENDPOINT:
        if ((uint8_t)USB_ControlRequest.wValue == FEATURE_SEL_EndpointHalt)
        {
            uint8_t EndpointIndex = ((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);

            if (EndpointIndex == ENDPOINT_CONTROLEP)
                return;

            selectEndpoint(EndpointIndex);

            if (UECONX & 1<<EPEN)
            {
                if (USB_ControlRequest.bRequest == REQ_SetFeature)
                {
                    UECONX |= 1<<STALLRQ;
                }
                else
                {
                    UECONX |= 1<<STALLRQC;
                    UERST = 1<<(EndpointIndex & ENDPOINT_EPNUM_MASK);
                    UERST = 0;
                    UECONX |= 1<<RSTDT;
                }
            }
        }

        break;
    default:
        return;
    }

    selectEndpoint(ENDPOINT_CONTROLEP);
    UEINTX &= ~(1<<RXSTPI);
    Endpoint_ClearStatusStage();
}

uint8_t USB::write_Control_Stream_LE(const void* const Buffer, uint16_t len)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    bool LastPacketFull = false;

    if (len > USB_ControlRequest.wLength)
        len = USB_ControlRequest.wLength;
    else if (!(len))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in

    while (len || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOINT_RWCSTREAM_HostAborted;
        else if (UEINTX & 1<<RXOUTI)
            break;

        if (UEINTX & 1<<TXINI)
        {
            uint16_t BytesInEndpoint = bytesInEndpoint();

            while (len && BytesInEndpoint < _control.size)
            {
                write8(*DataStream);
                DataStream++;
                len--;
                BytesInEndpoint++;
            }

            LastPacketFull = BytesInEndpoint == _control.size;
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
        }
    }

    while ((UEINTX & 1<<RXOUTI) == 0)
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t USB::write_Control_PStream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    bool LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOINT_RWCSTREAM_HostAborted;
        else if (UEINTX & 1<<RXOUTI)
            break;

        if (UEINTX & 1<<TXINI)
        {
            uint16_t BytesInEndpoint = bytesInEndpoint();

            while (Length && BytesInEndpoint < _control.size)
            {
                write8(pgm_read_byte(DataStream));
                DataStream++;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = BytesInEndpoint == _control.size;
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
        }
    }

    while (!(UEINTX & 1<<RXOUTI))
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t USB::Endpoint_BytesToEPSizeMask(const uint16_t Bytes)
{
    uint8_t  MaskVal    = 0;
    uint16_t CheckBytes = 8;

    while (CheckBytes < Bytes)
    {
        MaskVal++;
        CheckBytes <<= 1;
    }

    return MaskVal<<EPSIZE0;
}

bool USB::configureEndpoint(Endpoint &ep)
{
    configureEndpoint(ep.addr, ep.type, ep.size, ep.banks);
}

uint8_t USB::nullStream(uint16_t len, uint16_t * const bytesProcessed)
{
    uint8_t errCode = Endpoint_WaitUntilReady();
    uint16_t bytesInTransfer = 0;
    
    if (errCode)
        return errCode;

    if (bytesProcessed != NULL)
        len -= *bytesProcessed;

    while (len)
    {
        if ((UEINTX & 1<<RWAL) == 0) // read-write not allowed?
        {
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            
            if (bytesProcessed != NULL)
            {
                *bytesProcessed += bytesInTransfer;
                return ENDPOINT_RWSTREAM_IncompleteTransfer;
            }

            errCode = Endpoint_WaitUntilReady();

            if (errCode)
                return errCode;
        }
        else
        {
            write8(0);
            len--;
            bytesInTransfer++;
        }
    }
    
    return ENDPOINT_RWSTREAM_NoError;
}

uint8_t USB::readStream(void * const buf, uint16_t len, uint16_t * const bytes)
{
    uint8_t *dataStream = (uint8_t *)buf;
    uint16_t bytesInTransfer = 0;
    uint8_t errorCode = Endpoint_WaitUntilReady();

    if (errorCode)
        return errorCode;

    if (bytes != NULL)
    {
        len -= *bytes;
        dataStream += *bytes;
    }

    while (len)
    {
        if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
        {
            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out

            if (bytes != NULL)
            {
                *bytes += bytesInTransfer;
                return ENDPOINT_RWSTREAM_IncompleteTransfer;
            }
            
            errorCode = Endpoint_WaitUntilReady();

            if (errorCode)
                return errorCode;
        }
        else
        {
            *dataStream = read8();
            dataStream += 1;
            len--;
            bytesInTransfer++;
        }
    }

    return ENDPOINT_RWSTREAM_NoError;

}

uint8_t USB::writeStream(const void * const buf, uint16_t len, uint16_t * const bytes)
{
    uint8_t *dataStream = (uint8_t *)buf;
    uint16_t bytesInTransfer = 0;
    uint8_t errorCode;

    if (errorCode = Endpoint_WaitUntilReady())
        return errorCode;

    if (bytes != NULL)
    {
        len -= *bytes;
        dataStream += *bytes;
    }

    while (len)
    {
        if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
        {
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
            
            if (bytes != NULL)
            {
                *bytes += bytesInTransfer;
                return ENDPOINT_RWSTREAM_IncompleteTransfer;
            }

            if (errorCode = Endpoint_WaitUntilReady())
                return errorCode;
        }
        else
        {
            write8(*dataStream);
            *dataStream += 1;
            len--;
            bytesInTransfer++;
        }
    }

    return ENDPOINT_RWSTREAM_NoError;
}




