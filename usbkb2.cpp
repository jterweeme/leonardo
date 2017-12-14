#include "usbkb2.h"
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "misc.h"
#include <stdio.h>

const DescDev PROGMEM DeviceDescriptor =
{
    {
        sizeof(DescDev),
        DTYPE_Device
    },
    0x0110,
    USB_CSCP_NoDeviceClass,
    USB_CSCP_NoSpecificSubclass,
    USB_CSCP_NoSpecificProtocol,
    FIXED_CONTROL_ENDPOINT_SIZE,
    0x03EB,
    0x204B,
    0x0001,
    0x01,
    0x02,
    USE_INTERNAL_SERIAL,
    FIXED_NUM_CONFIGURATIONS
};

#if 0
const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
    {
        {
            sizeof(DescConf),
            DTYPE_Configuration
        },

        sizeof(USB_Descriptor_Configuration_t),
        2,
        1,
        NO_DESCRIPTOR,
        USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED,
        USB_CONFIG_POWER_MA(100)
    },
    {
        {
            sizeof(DescIface),
            DTYPE_Interface
        },
        .InterfaceNumber        = 0,
        .AlternateSetting       = 0,
        .TotalEndpoints         = 1,
        .Class                  = CDC_CSCP_CDCClass,
        .SubClass               = CDC_CSCP_ACMSubclass,
        .Protocol               = CDC_CSCP_ATCommandProtocol,
        .InterfaceStrIndex      = NO_DESCRIPTOR
    },
    {
        {
            sizeof(USB_CDC_Descriptor_FunctionalHeader_t),
            DTYPE_CSInterface
        },
        CDC_DSUBTYPE_CSInterface_Header,
        0x0110,
    },
    {
        {
            sizeof(USB_CDC_Descriptor_FunctionalACM_t),
            DTYPE_CSInterface
        },
        CDC_DSUBTYPE_CSInterface_ACM,
        0x06,
    },
    {
        {
            sizeof(USB_CDC_Descriptor_FunctionalUnion_t),
            DTYPE_CSInterface
        },
        CDC_DSUBTYPE_CSInterface_Union,
        0,
        1,
    },
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },
        CDC_NOTIFICATION_EPADDR,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        CDC_NOTIFICATION_EPSIZE,
        0xFF
    },
    {
        {
            sizeof(DescIface),
            DTYPE_Interface
        },
        1,
        0,
        2,
        CDC_CSCP_CDCDataClass,
        CDC_CSCP_NoDataSubclass,
        CDC_CSCP_NoDataProtocol,
        NO_DESCRIPTOR
    },
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },
        CDC_RX_EPADDR,
        EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        CDC_TXRX_EPSIZE,
        0x05
    },
};

const USB_Descriptor_String_t<2> PROGMEM LanguageString =
{
    {
        USB_STRING_LEN(1),
        DTYPE_String
    },
    (wchar_t)0x0409
};
#endif

const USB_Descriptor_String_t<12> PROGMEM ManufacturerString =
{
    {
        USB_STRING_LEN(11),
        DTYPE_String
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

uint16_t USBKB::getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddress)
{
    const uint8_t DescriptorType = wValue>>8;
    const uint8_t DescriptorNumber = wValue & 0xFF;
    const void* Address = NULL;
    uint16_t    Size    = NO_DESCRIPTOR;

    switch (DescriptorType)
    {
        case DTYPE_Device:
            Address = &DeviceDescriptor;
            Size = sizeof(DescDev);
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
                    Size = pgm_read_byte(&LanguageString.Header.size);
                    break;
                case 0x01:
                    Address = &ManufacturerString;
                    Size = pgm_read_byte(&ManufacturerString.Header.size);
                    break;
                case 0x02:
                    Address = &ProductString;
                    Size = pgm_read_byte(&ProductString.Header.size);
                    break;
            }

            break;
    }

    *descAddress = Address;
    return Size;
}

bool USBKB::configureEndpoints()
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

uint8_t USBKB::sendByte(uint8_t Data)
{
    if (GPIOR0 != DEVICE_STATE_Configured)
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

uint8_t USBKB::flush()
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

int16_t USBKB::receive()
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

USBKB::USBKB() :
    _inpoint(CDC_TX_EPADDR, CDC_TXRX_EPSIZE, EP_TYPE_BULK, 0),
    _outpoint(CDC_RX_EPADDR, CDC_TXRX_EPSIZE, EP_TYPE_BULK, 0),
    _notif(CDC_NOTIFICATION_EPADDR, CDC_NOTIFICATION_EPSIZE, EP_TYPE_INTERRUPT, 0)
{
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
    configureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL, _control.size, 1);
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

    if (UEINTX & 1<<RXSTPI)
        Device_ProcessControlRequest();

    selectEndpoint(PrevEndpoint);
}

ISR(USB_GEN_vect)
{
    USBKB::instance->gen();
}

void USBKB::gen()
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE) // start of frame
        UDINT &= ~(1<<SOFI);

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)   // usb connect
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
            {
                PLLCSR = USB_PLL_PSC;
                PLLCSR = USB_PLL_PSC | 1<<PLLE;
                while (!(PLLCSR & 1<<PLOCK));
            }

            GPIOR0 = DEVICE_STATE_Powered;
        }
        else    // disconnect
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
                PLLCSR = 0;

            GPIOR0 = DEVICE_STATE_Unattached;
        }
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<SUSPE)  // suspend
    {
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPI;
        USBCON |= 1<<FRZCLK;

        if (!(USB_Options & USB_OPT_MANUAL_PLL))
            PLLCSR = 0;

        GPIOR0 = DEVICE_STATE_Suspended;
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)   // wakeup
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
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE) // reset
    {
        UDINT &= ~(1<<EORSTI);
        GPIOR0 = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        configureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL, _control.size, 1);
        UEIENX |= 1<<RXSTPE;
    }
}

ISR(USB_COM_vect)
{
    USBKB::instance->com();
}

void USBKB::com()
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

bool USBKB::Endpoint_ConfigureEndpointTable(Endpoint *table, uint8_t entries)
{
    for (uint8_t i = 0; i < entries; i++)
    {
        if (!(table[i].addr))
            continue;

        if (!(configureEndpoint(table[i].addr, table[i].type, table[i].size, table[i].banks)))
            return false;
    }

    return true;
}

void USBKB::EVENT_USB_Device_ControlRequest()
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

void USBKB::Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t i = 0; i < sizeof(USB_Request_Header_t); i++)
        RequestHeader[i] = read8();

    char buf[50];
    snprintf(buf, 50, "%u\r\n", USB_ControlRequest.bRequest);
    Serial::instance->write(buf);
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
                        DescHeader Header;
                        uint16_t UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
                    }
                    sigDesc;

                    sigDesc.Header.type = DTYPE_String;
                    sigDesc.Header.size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
                    Device_GetSerialString(sigDesc.UnicodeString);
                    UEINTX &= ~(1<<RXSTPI);
                    Endpoint_Write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((descSize = getDescriptor(USB_ControlRequest.wValue,
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
                ConfigSuccess &= configureEndpoints();
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









