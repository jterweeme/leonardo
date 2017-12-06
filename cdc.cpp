#include "cdc.h"

#define USB_STRING_DESCRIPTOR(String) { .Header = {.Size = sizeof(USB_Descriptor_Header_t) + (sizeof(String) - 2), .Type = DTYPE_String}, String }

CDC *CDC::instance;

const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
    {
        .Size = sizeof(USB_Descriptor_Device_t),
        .Type = DTYPE_Device
    },
    VERSION_BCD(01.10),
    CDC_CSCP_CDCClass,
    CDC_CSCP_NoSpecificSubclass,
    CDC_CSCP_NoSpecificProtocol,
    FIXED_CONTROL_ENDPOINT_SIZE,
    .VendorID               = 0x03EB,
    .ProductID              = 0x204B,
    .ReleaseNumber          = VERSION_BCD(00.01),
    .ManufacturerStrIndex   = 0x01,
    .ProductStrIndex        = 0x02,
    .SerialNumStrIndex      = USE_INTERNAL_SERIAL,
    .NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
    .Config =
    {
        {
            .Size = sizeof(USB_Descriptor_Configuration_Header_t),
            .Type = DTYPE_Configuration
        },

        sizeof(USB_Descriptor_Configuration_t),
        2,
        1,
        .ConfigurationStrIndex  = NO_DESCRIPTOR,
        .ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),
        .MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
    },

    .CDC_CCI_Interface =
    {
        .Header =
        {
            .Size = sizeof(USB_Descriptor_Interface_t),
            .Type = DTYPE_Interface
        },
        .InterfaceNumber        = 0,
        .AlternateSetting       = 0,
        .TotalEndpoints         = 1,
        .Class                  = CDC_CSCP_CDCClass,
        .SubClass               = CDC_CSCP_ACMSubclass,
        .Protocol               = CDC_CSCP_ATCommandProtocol,
        .InterfaceStrIndex      = NO_DESCRIPTOR
    },

    .CDC_Functional_Header =
    {
        .Header =
        {
            .Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t),
            .Type = DTYPE_CSInterface
        },
        .Subtype      = CDC_DSUBTYPE_CSInterface_Header,
        .CDCSpecification       = VERSION_BCD(01.10),
    },

    .CDC_Functional_ACM =
    {
        .Header =
        {
            .Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t),
            .Type = DTYPE_CSInterface
        },
        .Subtype = CDC_DSUBTYPE_CSInterface_ACM,
        .Capabilities = 0x06,
    },

    .CDC_Functional_Union =
    {
        .Header =
        {
            .Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t),
            .Type = DTYPE_CSInterface
        },
        .Subtype = CDC_DSUBTYPE_CSInterface_Union,
        .MasterInterfaceNumber  = 0,
        .SlaveInterfaceNumber   = 1,
    },

    .CDC_NotificationEndpoint =
    {
        .Header =
        {
            .Size = sizeof(USB_Descriptor_Endpoint_t),
            .Type = DTYPE_Endpoint
        },
        .EndpointAddress        = CDC_NOTIFICATION_EPADDR,
        .Attributes     = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
        .EndpointSize           = CDC_NOTIFICATION_EPSIZE,
        .PollingIntervalMS      = 0xFF
    },

    .CDC_DCI_Interface =
        {
            .Header =
            {
                .Size = sizeof(USB_Descriptor_Interface_t),
                .Type = DTYPE_Interface
            },
            .InterfaceNumber   = 1,
            .AlternateSetting  = 0,
            .TotalEndpoints    = 2,
            .Class             = CDC_CSCP_CDCDataClass,
            .SubClass          = CDC_CSCP_NoDataSubclass,
            .Protocol          = CDC_CSCP_NoDataProtocol,
            .InterfaceStrIndex = NO_DESCRIPTOR
        },

    .CDC_DataOutEndpoint =
    {
        .Header =
        {
            .Size = sizeof(USB_Descriptor_Endpoint_t),
            .Type = DTYPE_Endpoint
        },
        .EndpointAddress   = CDC_RX_EPADDR,
        .Attributes        = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
        .EndpointSize      = CDC_TXRX_EPSIZE,
        .PollingIntervalMS = 0x05
    },

    .CDC_DataInEndpoint =
    {
        .Header =
        {
            .Size = sizeof(USB_Descriptor_Endpoint_t),
            .Type = DTYPE_Endpoint
        },
        .EndpointAddress        = CDC_TX_EPADDR,
        .Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
        .EndpointSize           = CDC_TXRX_EPSIZE,
        .PollingIntervalMS      = 0x05
    }
};

const USB_Descriptor_String_t<2> PROGMEM LanguageString =
{
    .Header =
    {
        .Size = USB_STRING_LEN(1),
        .Type = DTYPE_String
    },
    (wchar_t)0x0409
};

const USB_Descriptor_String_t<12> PROGMEM ManufacturerString =
{
    {
        .Size = USB_STRING_LEN(11),
        .Type = DTYPE_String
    },
    L"Dean Camera"
};

const USB_Descriptor_String_t<23> PROGMEM ProductString =
{
    {
        USB_STRING_LEN(22),
        DTYPE_String
    },
    L"LUFA USB-RS232 Adapter"
};

uint16_t CDC::CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                     const uint8_t wIndex, const void** const DescriptorAddress)
{
    const uint8_t DescriptorType = wValue>>8;
    const uint8_t DescriptorNumber = (wValue & 0xFF);
    const void* Address = NULL;
    uint16_t    Size    = NO_DESCRIPTOR;

    switch (DescriptorType)
    {
        case DTYPE_Device:
            Address = &DeviceDescriptor;
            Size = sizeof(USB_Descriptor_Device_t);
            break;
        case DTYPE_Configuration:
            Address = &ConfigurationDescriptor;
            Size = sizeof(USB_Descriptor_Configuration_t);
            break;
        case DTYPE_String:
            switch (DescriptorNumber)
            {
                case 0x00:
                    Address = &LanguageString;
                    Size = pgm_read_byte(&LanguageString.Header.Size);
                    break;
                case 0x01:
                    Address = &ManufacturerString;
                    Size = pgm_read_byte(&ManufacturerString.Header.Size);
                    break;
                case 0x02:
                    Address = &ProductString;
                    Size = pgm_read_byte(&ProductString.Header.Size);
                    break;
            }

            break;
    }
   *DescriptorAddress = Address;
    return Size;
}

bool CDC::CDC_Device_ConfigureEndpoints()
{
    memset(&_lineEncoding, 0, sizeof(_lineEncoding));

    if (!(Endpoint_ConfigureEndpointTable(&_inpoint, 1)))
        return false;

    if (!(Endpoint_ConfigureEndpointTable(&_outpoint, 1)))
        return false;

    if (!(Endpoint_ConfigureEndpointTable(&_notif, 1)))
        return false;

    return true;
}

uint8_t CDC::sendByte(uint8_t Data)
{
    if ((GPIOR0 != DEVICE_STATE_Configured) || !_lineEncoding.BaudRateBPS)
        return ENDPOINT_RWSTREAM_DeviceDisconnected;

    _inpoint.select();

    if (!(UEINTX & 1<<RWAL))
    {
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
        uint8_t ErrorCode;

        if ((ErrorCode = Endpoint_WaitUntilReady()) != ENDPOINT_READYWAIT_NoError)
            return ErrorCode;
    }

    write8(Data);
    return ENDPOINT_READYWAIT_NoError;
}

uint8_t CDC::flush()
{
    if ((GPIOR0 != DEVICE_STATE_Configured) || !_lineEncoding.BaudRateBPS)
        return ENDPOINT_RWSTREAM_DeviceDisconnected;

    uint8_t ErrorCode;
    _inpoint.select();

    if (!(bytesInEndpoint()))
        return ENDPOINT_READYWAIT_NoError;

    bool BankFull = !(UEINTX & 1<<RWAL);
    UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    if (BankFull)
    {
        if ((ErrorCode = Endpoint_WaitUntilReady()) != ENDPOINT_READYWAIT_NoError)
            return ErrorCode;

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
    }

    return ENDPOINT_READYWAIT_NoError;
}

int16_t CDC::receive()
{
    if ((GPIOR0 != DEVICE_STATE_Configured) || !(_lineEncoding.BaudRateBPS))
        return -1;

    int16_t ReceivedByte = -1;
    _outpoint.select();

    if (UEINTX & 1<<RXOUTI) // is OUT received?
    {
        if (bytesInEndpoint())
            ReceivedByte = read8();

        if (!(bytesInEndpoint()))
            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }

    return ReceivedByte;
}

CDC::CDC() :
    _control(ENDPOINT_CONTROLEP, USB_Device_ControlEndpointSize, EP_TYPE_CONTROL, 1),
    _inpoint(CDC_TX_EPADDR, CDC_TXRX_EPSIZE, EP_TYPE_BULK, 0),
    _outpoint(CDC_RX_EPADDR, CDC_TXRX_EPSIZE, EP_TYPE_BULK, 0),
    _notif(CDC_NOTIFICATION_EPADDR, CDC_NOTIFICATION_EPSIZE, EP_TYPE_INTERRUPT, 0)
{
    instance = this;
    MCUSR &= ~(1<<WDRF);
    wdt_disable();
    clock_prescale_set(clock_div_2);
    USBCON &= ~(1<<OTGPADE);

    if (!(USB_Options & USB_OPT_REG_DISABLED))
        UHWCON |= 1<<UVREGE;
    else
        UHWCON &= ~(1<<UVREGE);

    if (!(USB_Options & USB_OPT_MANUAL_PLL))
        PLLFRQ = 1<<PDIV2;

    USB_IsInitialized = true;
    USBCON &= ~(1<<VBUSTE);
    UDIEN = 0;
    USBINT = 0;
    USBCON &= ~(1<<USBE);
    USBCON |= 1<<USBE;
    USBCON &= ~(1<<FRZCLK);

    if (!(USB_Options & USB_OPT_MANUAL_PLL))
        PLLCSR = 0;

    GPIOR0 = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;

    if (USB_Options & USB_DEVICE_OPT_LOWSPEED)
        UDCON |= 1<<LSM;
    else
        UDCON &= ~(1<<LSM);

    USBCON |= 1<<VBUSTE;

    Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL,
                        USB_Device_ControlEndpointSize, 1);

    UDINT &= ~(1<<SUSPI);
    UDIEN |= 1<<SUSPE;
    UDIEN |= 1<<EORSTE;
    UDCON &= ~(1<<DETACH);
    USBCON |= 1<<OTGPADE;
    sei();

    if ((GPIOR0 != DEVICE_STATE_Configured) || !(_lineEncoding.BaudRateBPS))
        return;

    _inpoint.select();

    if (UEINTX & 1<<TXINI)
        flush();

    if (GPIOR0 == DEVICE_STATE_Unattached)
        return;

    uint8_t PrevEndpoint = getCurrentEndpoint();
    _control.select();
    //selectEndpoint(ENDPOINT_CONTROLEP);

    if (UEINTX & 1<<RXSTPI)
        Device_ProcessControlRequest();

    selectEndpoint(PrevEndpoint);
}

bool CDC::Endpoint_ConfigureEndpoint(const uint8_t Address,
                         const uint8_t Type, const uint16_t Size, const uint8_t Banks)
{
    uint8_t Number = Address & ENDPOINT_EPNUM_MASK;

    if (Number >= ENDPOINT_TOTAL_ENDPOINTS)
        return false;

    uint8_t cfg0 = Type << EPTYPE0 | ((Address & ENDPOINT_DIR_IN) ? 1<<EPDIR : 0);
    uint8_t cfg1 = 1<<ALLOC | ((Banks > 1) ? 1<<EPBK0 : 0) | Endpoint_BytesToEPSizeMask(Size);

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

uint8_t CDC::Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    bool LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = GPIOR0;

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

            while (Length && (BytesInEndpoint < USB_Device_ControlEndpointSize))
            {
                write8(*DataStream);
                DataStream++;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = BytesInEndpoint == USB_Device_ControlEndpointSize;
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
        }
    }

    while (!(UEINTX & 1<<RXOUTI))
    {
        uint8_t USB_DeviceState_LCL = GPIOR0;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t CDC::Endpoint_Write_Control_PStream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    bool LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = GPIOR0;

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

            while (Length && (BytesInEndpoint < USB_Device_ControlEndpointSize))
            {
                write8(pgm_read_byte(DataStream));
                DataStream++;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = (BytesInEndpoint == USB_Device_ControlEndpointSize);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
        }
    }

    while (!(UEINTX & 1<<RXOUTI))
    {
        uint8_t USB_DeviceState_LCL = GPIOR0;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

ISR(USB_GEN_vect)
{
    CDC *instance = CDC::instance;
    instance->gen();
}

void CDC::gen()
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE)
    {
        UDINT &= ~(1<<SOFI);
        //EVENT_USB_Device_StartOfFrame();
    }

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
            {
                PLLCSR = USB_PLL_PSC; PLLCSR = USB_PLL_PSC | 1<<PLLE;
                while (!(PLLCSR & 1<<PLOCK));
            }

            GPIOR0 = DEVICE_STATE_Powered;
            //EVENT_USB_Device_Connect();
        }
        else
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
                PLLCSR = 0;

            GPIOR0 = DEVICE_STATE_Unattached;
            //EVENT_USB_Device_Disconnect();
        }
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<SUSPE)
    {
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPI;
        USBCON |= 1<<FRZCLK;

        if (!(USB_Options & USB_OPT_MANUAL_PLL))
            PLLCSR = 0;

        GPIOR0 = DEVICE_STATE_Suspended;
        //EVENT_USB_Device_Suspend();
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)
    {
        if (!(USB_Options & USB_OPT_MANUAL_PLL))
        {
            PLLCSR = USB_PLL_PSC; PLLCSR = USB_PLL_PSC | 1<<PLLE;   // PLL on
            while (!(PLLCSR & 1<<PLOCK));   // PLL is ready?
        }

        USBCON &= ~(1<<FRZCLK);
        UDINT &= ~(1<<WAKEUPI);
        UDIEN &= ~(1<<WAKEUPI);
        UDIEN |= 1<<SUSPE;

        if (USB_Device_ConfigurationNumber)
            GPIOR0 = DEVICE_STATE_Configured;
        else
            GPIOR0 = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

        //EVENT_USB_Device_WakeUp();
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);
        GPIOR0 = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;

        Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL,
                            USB_Device_ControlEndpointSize, 1);

        UEIENX |= 1<<RXSTPE;
        //EVENT_USB_Device_Reset();
    }
}

ISR(USB_COM_vect)
{
    CDC *instance = CDC::instance;
    instance->com();
}

void CDC::com()
{
    uint8_t PrevSelectedEndpoint = getCurrentEndpoint();
    _control.select();
    UEIENX &= ~(1<<RXSTPE);
    sei();
    Device_ProcessControlRequest();
    _control.select();
    UEIENX |= 1<<RXSTPE;
    selectEndpoint(PrevSelectedEndpoint);
}

bool CDC::Endpoint_ConfigureEndpointTable(Endpoint *table, uint8_t entries)
{
    for (uint8_t i = 0; i < entries; i++)
    {
        if (!(table[i].addr))
            continue;

        if (!(Endpoint_ConfigureEndpoint(table[i].addr, table[i].Type,
                    table[i].Size, table[i].Banks)))
        {
            return false;
        }
    }

    return true;
}

uint8_t CDC::getEndpointDirection()
{
    return UECFG0X & 1<<EPDIR ? ENDPOINT_DIR_IN : ENDPOINT_DIR_OUT;
}

void CDC::Endpoint_ClearStatusStage()
{
    if (USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST)
    {
        while (!(UEINTX & 1<<RXOUTI))
            if (GPIOR0 == DEVICE_STATE_Unattached)
                return;

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }
    else
    {
        while (!(UEINTX & 1<<TXINI))
            if (GPIOR0 == DEVICE_STATE_Unattached)
                return;

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
    }
}

uint8_t CDC::Endpoint_WaitUntilReady()
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

        uint8_t USB_DeviceState_LCL = GPIOR0;

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

void CDC::EVENT_USB_Device_ControlRequest()
{
    if (!(UEINTX & 1<<RXSTPI))
        return;

    if (USB_ControlRequest.wIndex != _control.addr)
        return;

    switch (USB_ControlRequest.bRequest)
    {
        case CDC_REQ_GetLineEncoding:
            if (USB_ControlRequest.bmRequestType ==
                (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI);
                while (!(UEINTX & 1<<TXINI));
                write32(_lineEncoding.BaudRateBPS);
                write8(_lineEncoding.CharFormat);
                write8(_lineEncoding.ParityType);
                write8(_lineEncoding.DataBits);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
                Endpoint_ClearStatusStage();
            }
            break;
        case CDC_REQ_SetLineEncoding:
            if (USB_ControlRequest.bmRequestType ==
                    (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI);

                while (!(UEINTX & 1<<RXOUTI))
                    if (GPIOR0 == DEVICE_STATE_Unattached)
                        return;

                _lineEncoding.BaudRateBPS = read32();
                _lineEncoding.CharFormat = read8();
                _lineEncoding.ParityType = read8();
                _lineEncoding.DataBits = read8();
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                Endpoint_ClearStatusStage();
            }

            break;
        case CDC_REQ_SetControlLineState:
            if (USB_ControlRequest.bmRequestType ==
                    (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI);
                Endpoint_ClearStatusStage();
            }
            break;
        case CDC_REQ_SendBreak:
            if (USB_ControlRequest.bmRequestType ==
                    (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI);
                Endpoint_ClearStatusStage();
            }

            break;
    }
}

void CDC::Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t RequestHeaderByte = 0; RequestHeaderByte < sizeof(USB_Request_Header_t);
                   RequestHeaderByte++)
    {
        *(RequestHeader++) = read8();
    }

    EVENT_USB_Device_ControlRequest();

    if (UEINTX & 1<<RXSTPI)
    {
        uint8_t bmRequestType = USB_ControlRequest.bmRequestType;

        switch (USB_ControlRequest.bRequest)
        {
        case REQ_GetStatus:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                uint8_t CurrentStatus = 0;

                switch (USB_ControlRequest.bmRequestType)
                {
                    case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE):
                        if (USB_Device_CurrentlySelfPowered)
                            CurrentStatus |= FEATURE_SELFPOWERED_ENABLED;

                        if (USB_Device_RemoteWakeupEnabled)
                            CurrentStatus |= FEATURE_REMOTE_WAKEUP_ENABLED;

                        break;
                    case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT):
                        selectEndpoint((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);
                        CurrentStatus = UECONX & 1<<STALLRQ;
                        selectEndpoint(ENDPOINT_CONTROLEP);
                        break;
                    default:
                        return;
                }

                UEINTX &= ~(1<<RXSTPI);
                write16(CurrentStatus);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
                Endpoint_ClearStatusStage();
            }

            break;
        case REQ_ClearFeature:
        case REQ_SetFeature:
            if ((bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                Device_ClearSetFeature();
            }

            break;
        case REQ_SetAddress:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                uint8_t DeviceAddress = USB_ControlRequest.wValue & 0x7F;
                UDADDR = UDADDR & 1<<ADDEN | DeviceAddress & 0x7F;
                UEINTX &= ~(1<<RXSTPI);
                Endpoint_ClearStatusStage();
                while (!(UEINTX & 1<<TXINI));
                UDADDR |= 1<<ADDEN; // enable dev addr
                GPIOR0 = DeviceAddress ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
            }

            break;

        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                const void* DescriptorPointer;
                uint16_t descSize;

                if (USB_ControlRequest.wValue == (DTYPE_String << 8 | USE_INTERNAL_SERIAL))
                {
                    struct
                    {
                        USB_Descriptor_Header_t Header;
                        uint16_t UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
                    }
                    sigDesc;

                    sigDesc.Header.Type = DTYPE_String;
                    sigDesc.Header.Size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
                    Device_GetSerialString(sigDesc.UnicodeString);
                    UEINTX &= ~(1<<RXSTPI);
                    Endpoint_Write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((descSize = CALLBACK_USB_GetDescriptor(USB_ControlRequest.wValue,
                          USB_ControlRequest.wIndex, &DescriptorPointer)) == NO_DESCRIPTOR)
                {
                    return;
                }

                UEINTX &= ~(1<<RXSTPI);
                Endpoint_Write_Control_PStream_LE(DescriptorPointer, descSize);
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
            }

            break;
        case REQ_GetConfiguration:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                UEINTX &= ~(1<<RXSTPI);
                write8(USB_Device_ConfigurationNumber);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
                Endpoint_ClearStatusStage();
            }

            break;

        case REQ_SetConfiguration:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                if ((uint8_t)USB_ControlRequest.wValue > FIXED_NUM_CONFIGURATIONS)
                    return;

                UEINTX &= ~(1<<RXSTPI);
                USB_Device_ConfigurationNumber = (uint8_t)USB_ControlRequest.wValue;
                Endpoint_ClearStatusStage();

                if (USB_Device_ConfigurationNumber)
                    GPIOR0 = DEVICE_STATE_Configured;
                else
                    GPIOR0 = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

                bool ConfigSuccess = true;
                ConfigSuccess &= CDC_Device_ConfigureEndpoints();
            }
            break;
        default:
            break;
        }
    }

    if (UEINTX & 1<<RXSTPI)
    {
        UEINTX &= ~(1<<RXSTPI);     // clear setup
        UECONX |= 1<<STALLRQ;
    }
}

uint32_t CDC::read32()
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

void CDC::Device_ClearSetFeature()
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

void CDC::write32(uint32_t Data)
{
    UEDATX = Data &  0xFF;
    UEDATX = Data >> 8;
    UEDATX = Data >> 16;
    UEDATX = Data >> 24;
}

uint8_t CDC::Endpoint_BytesToEPSizeMask(const uint16_t Bytes)
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

void CDC::Device_GetSerialString(uint16_t * const UnicodeString)
{
    uint_reg_t CurrentGlobalInt = SREG;
    cli();
    uint8_t SigReadAddress = INTERNAL_SERIAL_START_ADDRESS;

    for (uint8_t SerialCharNum = 0;
        SerialCharNum < (INTERNAL_SERIAL_LENGTH_BITS / 4); SerialCharNum++)
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



