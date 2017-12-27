#include "cdc.h"
#include <avr/interrupt.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "misc.h"
#include <stdio.h>

static constexpr uint8_t
    CDC_DSUBTYPE_CSInterface_Header           = 0x00,
    CDC_DSUBTYPE_CSInterface_CallManagement   = 0x01,
    CDC_DSUBTYPE_CSInterface_ACM              = 0x02,
    CDC_DSUBTYPE_CSInterface_DirectLine       = 0x03,
    CDC_DSUBTYPE_CSInterface_TelephoneRinger  = 0x04,
    CDC_DSUBTYPE_CSInterface_TelephoneCall    = 0x05,
    CDC_DSUBTYPE_CSInterface_Union            = 0x06,
    CDC_DSUBTYPE_CSInterface_CountrySelection = 0x07,
    CDC_DSUBTYPE_CSInterface_TelephoneOpModes = 0x08,
    CDC_DSUBTYPE_CSInterface_USBTerminal      = 0x09,
    CDC_DSUBTYPE_CSInterface_NetworkChannel   = 0x0A,
    CDC_DSUBTYPE_CSInterface_ProtocolUnit     = 0x0B,
    CDC_DSUBTYPE_CSInterface_ExtensionUnit    = 0x0C,
    CDC_DSUBTYPE_CSInterface_MultiChannel     = 0x0D,
    CDC_DSUBTYPE_CSInterface_CAPI             = 0x0E,
    CDC_DSUBTYPE_CSInterface_Ethernet         = 0x0F,
    CDC_DSUBTYPE_CSInterface_ATM              = 0x10,
    CDC_NOTIFICATION_EPADDR = ENDPOINT_DIR_IN | 2,
    CDC_TX_EPADDR = ENDPOINT_DIR_IN | 3,
    CDC_RX_EPADDR = ENDPOINT_DIR_OUT | 4,
    CDC_NOTIFICATION_EPSIZE = 8,
    CDC_TXRX_EPSIZE = 16,
    CDC_PARITY_None  = 0,
    CDC_PARITY_Odd   = 1,
    CDC_PARITY_Even  = 2,
    CDC_PARITY_Mark  = 3,
    CDC_PARITY_Space = 4;


const DescDev PROGMEM DeviceDescriptor =
{
    {
        sizeof(DescDev),
        DTYPE_Device
    },
    0x0110,
    CDC_CSCP_CDCClass,
    CDC_CSCP_NoSpecificSubclass,
    CDC_CSCP_NoSpecificProtocol,
    FIXED_CONTROL_ENDPOINT_SIZE,
    0x03EB,
    0x204B,
    0x0001,
    0x01,
    0x02,
    USE_INTERNAL_SERIAL,
    FIXED_NUM_CONFIGURATIONS
};

struct USB_CDC_Descriptor_FunctionalHeader_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint16_t CDCSpecification;
}
__attribute__ ((packed));

struct USB_CDC_StdDescriptor_FunctionalHeader_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint16_t bcdCDC;
}
__attribute__ ((packed));

struct USB_CDC_Descriptor_FunctionalACM_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint8_t Capabilities;
}
__attribute__ ((packed));

struct USB_CDC_StdDescriptor_FunctionalACM_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bmCapabilities;
}
__attribute__ ((packed));

struct USB_CDC_Descriptor_FunctionalUnion_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint8_t MasterInterfaceNumber;
    uint8_t SlaveInterfaceNumber;
}
__attribute__ ((packed));

struct USB_CDC_StdDescriptor_FunctionalUnion_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bMasterInterface;
    uint8_t bSlaveInterface0;
}
__attribute__ ((packed));

struct MyConfiguration
{
    DescConf config;
    DescIface CDC_CCI_Interface;
    USB_CDC_Descriptor_FunctionalHeader_t    CDC_Functional_Header;
    USB_CDC_Descriptor_FunctionalACM_t       CDC_Functional_ACM;
    USB_CDC_Descriptor_FunctionalUnion_t     CDC_Functional_Union;
    DescEndpoint CDC_NotificationEndpoint;
    DescIface CDC_DCI_Interface;
    DescEndpoint CDC_DataOutEndpoint;
    DescEndpoint CDC_DataInEndpoint;
}
__attribute__ ((packed));

const MyConfiguration PROGMEM ConfigurationDescriptor =
{
    {
        {
            sizeof(DescConf),
            DTYPE_Configuration
        },

        sizeof(MyConfiguration),
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
        0,
        0,
        1,
        CDC_CSCP_CDCClass,
        CDC_CSCP_ACMSubclass,
        CDC_CSCP_ATCommandProtocol,
        NO_DESCRIPTOR
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
        (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
        CDC_TXRX_EPSIZE,
        0x05
    },
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },
        CDC_TX_EPADDR,
        (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
        CDC_TXRX_EPSIZE,
        0x05
    }
};

const USB_Descriptor_String_t<2> PROGMEM LanguageString =
{
    {
        USB_STRING_LEN(1),
        DTYPE_String
    },
    (wchar_t)0x0409
};

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

uint16_t CDC::getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddress)
{
    const uint8_t descType = wValue >> 8;
    const uint8_t descNumber = wValue & 0xFF;
    const void* addr = NULL;
    uint16_t size = NO_DESCRIPTOR;

    switch (descType)
    {
        case DTYPE_Device:
            addr = &DeviceDescriptor;
            size = sizeof(DescDev);
            break;
        case DTYPE_Configuration:
            addr = &ConfigurationDescriptor;
            size = sizeof(ConfigurationDescriptor);
            break;
        case DTYPE_String:
            switch (descNumber)
            {
                case 0x00:
                    addr = &LanguageString;
                    size = pgm_read_byte(&LanguageString.Header.size);
                    break;
                case 0x01:
                    addr = &ManufacturerString;
                    size = pgm_read_byte(&ManufacturerString.Header.size);
                    break;
                case 0x02:
                    addr = &ProductString;
                    size = pgm_read_byte(&ProductString.Header.size);
                    break;
            }

            break;
    }

    *descAddress = addr;
    return size;
}

uint8_t CDC::sendByte(uint8_t Data)
{
    if (state != DEVICE_STATE_Configured)
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
    if ((state != DEVICE_STATE_Configured) || !_lineEncoding.BaudRateBPS)
        return ENDPOINT_RWSTREAM_DeviceDisconnected;

    uint8_t ErrorCode;
    _inpoint.select();

    if (!(bytesInEndpoint()))
        return ENDPOINT_READYWAIT_NoError;

    bool bankFull = !(UEINTX & 1<<RWAL);
    UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    if (bankFull)
    {
        if ((ErrorCode = Endpoint_WaitUntilReady()) != ENDPOINT_READYWAIT_NoError)
            return ErrorCode;

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
    }

    return ENDPOINT_READYWAIT_NoError;
}

int16_t CDC::receive()
{
    if ((state != DEVICE_STATE_Configured) || !(_lineEncoding.BaudRateBPS))
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
    _inpoint(CDC_TX_EPADDR, CDC_TXRX_EPSIZE, EP_TYPE_BULK, 0),
    _outpoint(CDC_RX_EPADDR, CDC_TXRX_EPSIZE, EP_TYPE_BULK, 0),
    _notif(CDC_NOTIFICATION_EPADDR, CDC_NOTIFICATION_EPSIZE, EP_TYPE_INTERRUPT, 0)
{
    USBCON &= ~(1<<OTGPADE);
    UHWCON |= 1<<UVREGE;    // enable USB pad regulator
    USB_IsInitialized = true;
    USBCON &= ~(1<<VBUSTE);     // disable VBUS transition interrupt
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1<<USBE);   // disable USB controller clock inputs
    USBCON |= 1<<USBE;      // enable USB controller clock inputs
    USBCON &= ~(1<<FRZCLK); // enable clock inputs
    PLLCSR = 0;
    state = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    UDCON &= ~(1<<LSM);
    USBCON |= 1<<VBUSTE;    // enable VBUS transition interrupt
    configureEndpoint(_control.addr, _control.type, _control.size, 1);
    UDINT &= ~(1<<SUSPI);   // clear suspend interrupt flag
    UDIEN |= 1<<SUSPE;      // enable SUSPI interrupt
    UDIEN |= 1<<EORSTE;     // enable EORSTI interrupt
    UDCON &= ~(1<<DETACH);  // reconnect device
    USBCON |= 1<<OTGPADE;   // otgpad on
    sei();

    if ((state != DEVICE_STATE_Configured) || !(_lineEncoding.BaudRateBPS))
        return;

    _inpoint.select();

    if (UEINTX & 1<<TXINI)
        flush();

    if (state == DEVICE_STATE_Unattached)
        return;

    uint8_t PrevEndpoint = getCurrentEndpoint();
    _control.select();

    if (UEINTX & 1<<RXSTPI)
        Device_ProcessControlRequest();

    selectEndpoint(PrevEndpoint);
}

ISR(USB_GEN_vect)
{
    CDC::instance->gen();
}

void CDC::gen()
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

            state = DEVICE_STATE_Powered;
        }
        else    // disconnect
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
                PLLCSR = 0;

            state = DEVICE_STATE_Unattached;
        }
    }

    if (UDINT & 1<<SUSPI && UDIEN & 1<<SUSPE)  // suspend
    {
        UDIEN &= ~(1<<SUSPE);   // disable int suspi
        UDIEN |= 1<<WAKEUPE;    // enable int wakeupi
        USBCON |= 1<<FRZCLK;    // clk freeze

        if (!(USB_Options & USB_OPT_MANUAL_PLL))
            PLLCSR = 0;

        state = DEVICE_STATE_Suspended;
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)   // wakeup
    {
        if (!(USB_Options & USB_OPT_MANUAL_PLL))
        {
            PLLCSR = USB_PLL_PSC;
            PLLCSR = USB_PLL_PSC | 1<<PLLE;   // PLL on
            while (!(PLLCSR & 1<<PLOCK));   // PLL is ready?
        }

        USBCON &= ~(1<<FRZCLK); // clk unfreeze
        UDINT &= ~(1<<WAKEUPI);
        UDIEN &= ~(1<<WAKEUPI);
        UDIEN |= 1<<SUSPE;

        if (USB_Device_ConfigurationNumber)
            state = DEVICE_STATE_Configured;
        else
            state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE) // reset
    {
        UDINT &= ~(1<<EORSTI);
        state = DEVICE_STATE_Default;
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
    CDC::instance->com();
}

void CDC::com()
{
    uint8_t PrevSelectedEndpoint = getCurrentEndpoint();
    _control.select();
    UEIENX &= ~(1<<RXSTPE);     // disable int rxstpi
    sei();
    Device_ProcessControlRequest();
    _control.select();
    UEIENX |= 1<<RXSTPE;
    selectEndpoint(PrevSelectedEndpoint);
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
                    if (state == DEVICE_STATE_Unattached)
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

    for (uint8_t i = 0; i < sizeof(USB_Request_Header_t); i++)
        RequestHeader[i] = read8();

    EVENT_USB_Device_ControlRequest();

    if (UEINTX & 1<<RXSTPI) // endpoint isSetupReceived?
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
                state = DeviceAddress ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
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
                    write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((descSize = getDescriptor(USB_ControlRequest.wValue,
                          USB_ControlRequest.wIndex, &DescriptorPointer)) == NO_DESCRIPTOR)
                {
                    return;
                }

                UEINTX &= ~(1<<RXSTPI); // clear setup
                write_Control_PStream_LE(DescriptorPointer, descSize);
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
            }

            break;
        case REQ_GetConfiguration:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                write8(USB_Device_ConfigurationNumber);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
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
                    state = DEVICE_STATE_Configured;
                else
                    state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

                bool ConfigSuccess = true;
                //ConfigSuccess &= configureEndpoints();
                memset(&_lineEncoding, 0, sizeof(_lineEncoding));

                ConfigSuccess &= configureEndpoint(_inpoint.addr, _inpoint.type,
                    _inpoint.size, _inpoint.banks);

                ConfigSuccess &= configureEndpoint(_outpoint.addr, _outpoint.type,
                    _outpoint.size, _outpoint.banks);

                ConfigSuccess &= configureEndpoint(_notif.addr, _notif.type,
                    _notif.size, _notif.banks);
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









