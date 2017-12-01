#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "busby.h"

#define F_CPU 16000000UL

#define USB_CurrentMode USB_MODE_Device
#define USB_Device_ControlEndpointSize FIXED_CONTROL_ENDPOINT_SIZE

USB *USB::instance;

static const uint16_t LANGUAGE_ID_ENG = 0x0409;
static const uint8_t USB_CONFIG_ATTR_RESERVED = 0x80;
static const uint8_t USB_CONFIG_ATTR_SELFPOWERED = 0x40;
static const uint8_t USB_CONFIG_ATTR_REMOTEWAKEUP = 0x20;
static const uint8_t ENDPOINT_ATTR_NO_SYNC = 0 << 2;
static const uint8_t ENDPOINT_ATTR_ASYNC = 1 << 2;
static const uint8_t ENDPOINT_ATTR_ADAPTIVE = 2 << 2;
static const uint8_t ENDPOINT_ATTR_SYNC = 3 << 2;
static const uint8_t ENDPOINT_USAGE_DATA = 0 << 4;
static const uint8_t ENDPOINT_USAGE_FEEDBACK = 1 << 4;
static const uint8_t ENDPOINT_USAGE_IMPLICIT_FEEDBACK = 2 << 4;

static const uint8_t CONTROL_REQTYPE_DIRECTION = 0x80,
    CONTROL_REQTYPE_TYPE = 0x60,
    CONTROL_REQTYPE_RECIPIENT = 0x1f,
    REQDIR_HOSTTODEVICE = 0 << 7,
    REQDIR_DEVICETOHOST = 1 << 7,
    REQTYPE_STANDARD = 0 << 5,
    REQTYPE_CLASS = 1 << 5,
    REQTYPE_VENDOR = 2 << 5,
    REQREC_DEVICE = 0 << 0,
    REQREC_INTERFACE = 1 << 0,
    REQREC_ENDPOINT = 2 << 0,
    REQREC_OTHER = 3 << 0,
    FEATURE_SELFPOWERED_ENABLED = 1 << 0,
    FEATURE_REMOTE_WAKEUP_ENABLED = 1 << 1;

typedef struct
{
    USB_Descriptor_Header_t Header;
    uint8_t                 Subtype;
    uint16_t                CDCSpecification;
} ATTR_PACKED USB_CDC_Descriptor_FunctionalHeader_t;

typedef struct
{
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint16_t bcdCDC;
} ATTR_PACKED USB_CDC_StdDescriptor_FunctionalHeader_t;

typedef struct
{
    USB_Descriptor_Header_t Header;
    uint8_t                 Subtype;
    uint8_t                 Capabilities;
} ATTR_PACKED USB_CDC_Descriptor_FunctionalACM_t;


typedef struct
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bmCapabilities;
} ATTR_PACKED USB_CDC_StdDescriptor_FunctionalACM_t;

typedef struct
{
    USB_Descriptor_Header_t Header;
    uint8_t                 Subtype;
    uint8_t                 MasterInterfaceNumber;
    uint8_t                 SlaveInterfaceNumber;
} ATTR_PACKED USB_CDC_Descriptor_FunctionalUnion_t;

typedef struct
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bMasterInterface;
    uint8_t bSlaveInterface0;
} ATTR_PACKED USB_CDC_StdDescriptor_FunctionalUnion_t;

extern "C" {
void
EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
ATTR_NON_NULL_PTR_ARG(1);


void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
ATTR_NON_NULL_PTR_ARG(1);

}

void
CDC_Device_SendControlLineStateChange(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
ATTR_NON_NULL_PTR_ARG(1);

static const uint8_t CDC_NOTIFICATION_EPADDR = ENDPOINT_DIR_IN | 2,
    CDC_TX_EPADDR = ENDPOINT_DIR_IN | 3,
    CDC_RX_EPADDR = ENDPOINT_DIR_OUT | 4,
    CDC_NOTIFICATION_EPSIZE = 8,
    CDC_TXRX_EPSIZE = 16;


uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue, const uint8_t wIndex,
                                    const void** const DescriptorAddress)
                                    ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);


struct USB_Descriptor_Configuration_t
{
    USB_Descriptor_Configuration_Header_t    Config;
    USB_Descriptor_Interface_t               CDC_CCI_Interface;
    USB_CDC_Descriptor_FunctionalHeader_t    CDC_Functional_Header;
    USB_CDC_Descriptor_FunctionalACM_t       CDC_Functional_ACM;
    USB_CDC_Descriptor_FunctionalUnion_t     CDC_Functional_Union;
    USB_Descriptor_Endpoint_t                CDC_NotificationEndpoint;
    USB_Descriptor_Interface_t               CDC_DCI_Interface;
    USB_Descriptor_Endpoint_t                CDC_DataOutEndpoint;
    USB_Descriptor_Endpoint_t                CDC_DataInEndpoint;
} __attribute__ ((packed));


extern "C"
{
    void CDC_Device_Event_Stub(void) ATTR_CONST;

    void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
                        ATTR_WEAK ATTR_NON_NULL_PTR_ARG(1) ATTR_ALIAS(CDC_Device_Event_Stub);

    void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
                     ATTR_WEAK ATTR_NON_NULL_PTR_ARG(1) ATTR_ALIAS(CDC_Device_Event_Stub);
}

void USB::init()
{
    USB_OTGPAD_Off();

    if (!(USB_Options & USB_OPT_REG_DISABLED))
        USB_REG_On();
    else
        USB_REG_Off();

    if (!(USB_Options & USB_OPT_MANUAL_PLL))
        PLLFRQ = 1<<PDIV2;

    USB_IsInitialized = true;
    resetInterface();
}

void USB::resetInterface()
{
    disableAllInterrupts();
    clearAllInterrupts();
    USB_Controller_Reset();
    USB_CLK_Unfreeze();

    if (USB_CurrentMode == USB_MODE_Device)
    {
        if (!(USB_Options & USB_OPT_MANUAL_PLL))
            USB_PLL_Off();

        initDevice();
    }

    USB_OTGPAD_On();
}

static const uint8_t USB_DEVICE_OPT_LOWSPEED = 1<<0;

void USB::initDevice()
{
    USB_DeviceState = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;

    if (USB_Options & USB_DEVICE_OPT_LOWSPEED)
        USB_Device_SetLowSpeed();
    else
        USB_Device_SetFullSpeed();

    USB_INT_Enable(USB_INT_VBUSTI);

    Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL,
                        USB_Device_ControlEndpointSize, 1);

    USB_INT_Clear(USB_INT_SUSPI);
    USB_INT_Enable(USB_INT_SUSPI);
    USB_INT_Enable(USB_INT_EORSTI);
    USB_Attach();
}

void USB::CDC_Device_ProcessControlRequest(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
    if (!(USB::Endpoint_IsSETUPReceived()))
        return;

    if (USB_ControlRequest.wIndex != CDCInterfaceInfo->Config.ControlInterfaceNumber)
        return;

    switch (USB_ControlRequest.bRequest)
    {
        case CDC_REQ_GetLineEncoding:
            if (USB_ControlRequest.bmRequestType ==
                (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                Endpoint_ClearSETUP();
                while (!(USB::Endpoint_IsINReady()));
                Endpoint_Write_32_LE(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);
                Endpoint_Write_8(CDCInterfaceInfo->State.LineEncoding.CharFormat);
                Endpoint_Write_8(CDCInterfaceInfo->State.LineEncoding.ParityType);
                Endpoint_Write_8(CDCInterfaceInfo->State.LineEncoding.DataBits);
                Endpoint_ClearIN();
                Endpoint_ClearStatusStage();
            }
            break;
        case CDC_REQ_SetLineEncoding:
            if (USB_ControlRequest.bmRequestType ==
                    (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                Endpoint_ClearSETUP();

                while (!(USB::Endpoint_IsOUTReceived()))
                {
                    if (USB_DeviceState == DEVICE_STATE_Unattached)
                      return;
                }

                CDCInterfaceInfo->State.LineEncoding.BaudRateBPS = USB::Endpoint_Read_32_LE();
                CDCInterfaceInfo->State.LineEncoding.CharFormat  = USB::Endpoint_Read_8();
                CDCInterfaceInfo->State.LineEncoding.ParityType  = USB::Endpoint_Read_8();
                CDCInterfaceInfo->State.LineEncoding.DataBits    = USB::Endpoint_Read_8();
                Endpoint_ClearOUT();
                Endpoint_ClearStatusStage();
                EVENT_CDC_Device_LineEncodingChanged(CDCInterfaceInfo);
            }

            break;
        case CDC_REQ_SetControlLineState:
            if (USB_ControlRequest.bmRequestType ==
                    (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                USB::Endpoint_ClearSETUP();
                USB::Endpoint_ClearStatusStage();
                CDCInterfaceInfo->State.ControlLineStates.HostToDevice = USB_ControlRequest.wValue;
                EVENT_CDC_Device_ControLineStateChanged(CDCInterfaceInfo);
            }
            break;
        case CDC_REQ_SendBreak:
            if (USB_ControlRequest.bmRequestType ==
                    (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                USB::Endpoint_ClearSETUP();
                USB::Endpoint_ClearStatusStage();
            }

            break;
    }
}

bool USB::CDC_Device_ConfigureEndpoints(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
    memset(&CDCInterfaceInfo->State, 0x00, sizeof(CDCInterfaceInfo->State));
    CDCInterfaceInfo->Config.DataINEndpoint.Type = EP_TYPE_BULK;
    CDCInterfaceInfo->Config.DataOUTEndpoint.Type = EP_TYPE_BULK;
    CDCInterfaceInfo->Config.NotificationEndpoint.Type = EP_TYPE_INTERRUPT;

    if (!(USB::Endpoint_ConfigureEndpointTable(&CDCInterfaceInfo->Config.DataINEndpoint, 1)))
        return false;

    if (!(USB::Endpoint_ConfigureEndpointTable(&CDCInterfaceInfo->Config.DataOUTEndpoint, 1)))
        return false;

    if (!(USB::Endpoint_ConfigureEndpointTable(&CDCInterfaceInfo->Config.NotificationEndpoint, 1)))
        return false;

    return true;
}

void USB::CDC_Device_USBTask(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
    if ((USB_DeviceState != DEVICE_STATE_Configured) ||
            !(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS))
    {
        return;
    }

    Endpoint_SelectEndpoint(CDCInterfaceInfo->Config.DataINEndpoint.Address);

    if (Endpoint_IsINReady())
        CDC_Device_Flush(CDCInterfaceInfo);
}

uint8_t USB::CDC_Device_SendByte(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo,
                            const uint8_t Data)
{
    if ((USB_DeviceState != DEVICE_STATE_Configured) ||
        !(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS))
    {
        return ENDPOINT_RWSTREAM_DeviceDisconnected;
    }

    Endpoint_SelectEndpoint(CDCInterfaceInfo->Config.DataINEndpoint.Address);

    if (!(Endpoint_IsReadWriteAllowed()))
    {
        Endpoint_ClearIN();
        uint8_t ErrorCode;

        if ((ErrorCode = Endpoint_WaitUntilReady()) != ENDPOINT_READYWAIT_NoError)
            return ErrorCode;
    }

    Endpoint_Write_8(Data);
    return ENDPOINT_READYWAIT_NoError;
}

uint8_t USB::CDC_Device_Flush(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
    if ((USB_DeviceState != DEVICE_STATE_Configured) || !(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS))
      return ENDPOINT_RWSTREAM_DeviceDisconnected;

    uint8_t ErrorCode;

    Endpoint_SelectEndpoint(CDCInterfaceInfo->Config.DataINEndpoint.Address);

    if (!(Endpoint_BytesInEndpoint()))
      return ENDPOINT_READYWAIT_NoError;

    bool BankFull = !(Endpoint_IsReadWriteAllowed());

    Endpoint_ClearIN();

    if (BankFull)
    {
        if ((ErrorCode = Endpoint_WaitUntilReady()) != ENDPOINT_READYWAIT_NoError)
            return ErrorCode;

        Endpoint_ClearIN();
    }

    return ENDPOINT_READYWAIT_NoError;
}

int16_t USB::CDC_Device_ReceiveByte(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
    if ((USB_DeviceState != DEVICE_STATE_Configured) ||
            !(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS))
    {
        return -1;
    }

    int16_t ReceivedByte = -1;
    Endpoint_SelectEndpoint(CDCInterfaceInfo->Config.DataOUTEndpoint.Address);

    if (Endpoint_IsOUTReceived())
    {
        if (Endpoint_BytesInEndpoint())
            ReceivedByte = Endpoint_Read_8();

        if (!(Endpoint_BytesInEndpoint()))
            Endpoint_ClearOUT();
    }

    return ReceivedByte;
}

void USB::createStream(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo, FILE* const Stream)
{
    //*Stream = (FILE)FDEV_SETUP_STREAM(CDC_Device_putchar, CDC_Device_getchar, _FDEV_SETUP_RW);
    Stream->put = CDC_Device_putchar;
    Stream->get = CDC_Device_getchar;
    Stream->flags = _FDEV_SETUP_RW;
    Stream->udata = 0;
    fdev_set_udata(Stream, CDCInterfaceInfo);
}

int USB::CDC_Device_putchar(char c, FILE* Stream)
{
    return CDC_Device_SendByte((USB_ClassInfo_CDC_Device_t*)fdev_get_udata(Stream), c) ? _FDEV_ERR : 0;
}

int USB::CDC_Device_getchar(FILE* Stream)
{
    int16_t ReceivedByte = CDC_Device_ReceiveByte((USB_ClassInfo_CDC_Device_t*)fdev_get_udata(Stream));

    if (ReceivedByte < 0)
        return _FDEV_EOF;

    return ReceivedByte;
}

void CDC_Device_Event_Stub(void)
{
}

USB::USB()
{
    instance = this;
    MCUSR &= ~(1<<WDRF);
    wdt_disable();
    clock_prescale_set(clock_div_2);
    init();
    createStream();
    GlobalInterruptEnable();
    cdcTask();
    task();
}

void USB::myPutc(char c)
{
    CDC_Device_SendByte((USB_ClassInfo_CDC_Device_t*)fdev_get_udata(&cdcStream), c);
}

uint8_t USB::readByte()
{
    return CDC_Device_ReceiveByte((USB_ClassInfo_CDC_Device_t*)fdev_get_udata(&cdcStream));
}

USB_ClassInfo_CDC_Device_t USB::cdcDevice =
{
    {
        0,
        {CDC_TX_EPADDR, CDC_TXRX_EPSIZE, 1,},
        {CDC_RX_EPADDR, CDC_TXRX_EPSIZE, 1,},
        {CDC_NOTIFICATION_EPADDR, CDC_NOTIFICATION_EPSIZE, 1,},
    },
};

void USB::EVENT_USB_Device_ConfigurationChanged()
{
    bool ConfigSuccess = true;
    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&cdcDevice);
}

void USB::EVENT_USB_Device_ControlRequest()
{
    CDC_Device_ProcessControlRequest(&cdcDevice);
}

const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
    {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},
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
         {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},
             sizeof(USB_Descriptor_Configuration_t),
             2,
             1,
            .ConfigurationStrIndex  = NO_DESCRIPTOR,
            .ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),
            .MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
    },

    .CDC_CCI_Interface =
    {
        .Header      = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
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
    .Header = {.Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t), .Type = DTYPE_CSInterface},
        .Subtype      = CDC_DSUBTYPE_CSInterface_Header,
        .CDCSpecification       = VERSION_BCD(01.10),
    },

    .CDC_Functional_ACM =
    {
    .Header = {.Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t), .Type = DTYPE_CSInterface},
        .Subtype = CDC_DSUBTYPE_CSInterface_ACM,
        .Capabilities = 0x06,
    },

    .CDC_Functional_Union =
    {
    .Header = {.Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t), .Type = DTYPE_CSInterface},
        .Subtype = CDC_DSUBTYPE_CSInterface_Union,
        .MasterInterfaceNumber  = 0,
        .SlaveInterfaceNumber   = 1,
    },

    .CDC_NotificationEndpoint =
        {
            .Header      = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
            .EndpointAddress        = CDC_NOTIFICATION_EPADDR,
            .Attributes     = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
            .EndpointSize           = CDC_NOTIFICATION_EPSIZE,
            .PollingIntervalMS      = 0xFF
        },

    .CDC_DCI_Interface =
        {
            .Header  = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
            .InterfaceNumber        = 1,
            .AlternateSetting       = 0,
            .TotalEndpoints         = 2,
            .Class                  = CDC_CSCP_CDCDataClass,
            .SubClass               = CDC_CSCP_NoDataSubclass,
            .Protocol               = CDC_CSCP_NoDataProtocol,

            .InterfaceStrIndex      = NO_DESCRIPTOR
        },

    .CDC_DataOutEndpoint =
        {
            .Header        = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
            .EndpointAddress        = CDC_RX_EPADDR,
            .Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
            .EndpointSize           = CDC_TXRX_EPSIZE,
            .PollingIntervalMS      = 0x05
        },

    .CDC_DataInEndpoint =
    {
        .Header    = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
        .EndpointAddress        = CDC_TX_EPADDR,
        .Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
        .EndpointSize           = CDC_TXRX_EPSIZE,
        .PollingIntervalMS      = 0x05
    }
};

const USB_Descriptor_String_t PROGMEM LanguageString =
{
    .Header                 = {.Size = USB_STRING_LEN(1), .Type = DTYPE_String}
    //.UnicodeString          = (wchar_t)0x0409
};

const USB_Descriptor_String_t PROGMEM ManufacturerString =
{
    {.Size = USB_STRING_LEN(11), .Type = DTYPE_String},
     //L"Dean Camera"
};

const USB_Descriptor_String_t PROGMEM ProductString =
{
    .Header                 = {.Size = USB_STRING_LEN(22), .Type = DTYPE_String},
    //.UnicodeString          = L"LUFA USB-RS232 Adapter"
};

bool USB::Endpoint_ConfigureEndpoint(const uint8_t Address,
                         const uint8_t Type, const uint16_t Size, const uint8_t Banks)
{
    uint8_t Number = (Address & ENDPOINT_EPNUM_MASK);

    if (Number >= ENDPOINT_TOTAL_ENDPOINTS)
        return false;

    return Endpoint_ConfigureEndpoint_Prv(Number,
                       ((Type << EPTYPE0) | ((Address & ENDPOINT_DIR_IN) ? (1 << EPDIR) : 0)),
           ((1 << ALLOC) | ((Banks > 1) ? (1 << EPBK0) : 0) | Endpoint_BytesToEPSizeMask(Size)));
}

void USB::USB_Device_SetDeviceAddress(uint8_t Address)
{
    UDADDR = UDADDR & 1<<ADDEN | Address & 0x7F;
}


uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                     const uint8_t wIndex, const void** const DescriptorAddress)
{
    const uint8_t DescriptorType   = (wValue >> 8);
    const uint8_t DescriptorNumber = (wValue & 0xFF);

    const void* Address = NULL;
    uint16_t    Size    = NO_DESCRIPTOR;

    switch (DescriptorType)
    {
        case DTYPE_Device:
            Address = &DeviceDescriptor;
            Size    = sizeof(USB_Descriptor_Device_t);
            break;
        case DTYPE_Configuration:
            Address = &ConfigurationDescriptor;
            Size    = sizeof(USB_Descriptor_Configuration_t);
            break;
        case DTYPE_String:
            switch (DescriptorNumber)
            {
                case 0x00:
                    Address = &LanguageString;
                    Size    = pgm_read_byte(&LanguageString.Header.Size);
                    break;
                case 0x01:
                    Address = &ManufacturerString;
                    Size    = pgm_read_byte(&ManufacturerString.Header.Size);
                    break;
                case 0x02:
                    Address = &ProductString;
                    Size    = pgm_read_byte(&ProductString.Header.Size);
                    break;
            }

            break;
    }
   *DescriptorAddress = Address;
    return Size;
}

//static void USB_DeviceTask();
volatile bool USB_IsInitialized;
USB_Request_Header_t USB_ControlRequest;

void USB::USB_DeviceTask()
{
    if (USB_DeviceState == DEVICE_STATE_Unattached)
        return;

    uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();
    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);

    if (Endpoint_IsSETUPReceived())
        Device_ProcessControlRequest();

    USB::Endpoint_SelectEndpoint(PrevEndpoint);
}

uint8_t USB::Endpoint_Write_Stream_LE(const void * const Buffer,
                            uint16_t Length, uint16_t* const BytesProcessed)
{
    uint8_t *DataStream = (uint8_t*)Buffer;
    uint16_t BytesInTransfer = 0;
    uint8_t  ErrorCode;

    if ((ErrorCode = Endpoint_WaitUntilReady()))
        return ErrorCode;

    if (BytesProcessed != NULL)
    {
        Length -= *BytesProcessed;
        DataStream += *BytesProcessed;
    }

    while (Length)
    {
        if (!(USB::Endpoint_IsReadWriteAllowed()))
        {
            USB::Endpoint_ClearIN();

            if (BytesProcessed != NULL)
            {
                *BytesProcessed += BytesInTransfer;
                return ENDPOINT_RWSTREAM_IncompleteTransfer;
            }

            if ((ErrorCode = Endpoint_WaitUntilReady()))
                return ErrorCode;
        }
        else
        {
            USB::Endpoint_Write_8(*DataStream);
            DataStream += 1;
            Length--;
            BytesInTransfer++;
        }
    }

    return ENDPOINT_RWSTREAM_NoError;
}

uint8_t USB::Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    bool LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        Endpoint_ClearIN();

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (USB::Endpoint_IsSETUPReceived())
            return ENDPOINT_RWCSTREAM_HostAborted;
        else if (USB::Endpoint_IsOUTReceived())
            break;

        if (USB::Endpoint_IsINReady())
        {
            uint16_t BytesInEndpoint = Endpoint_BytesInEndpoint();

            while (Length && (BytesInEndpoint < USB_Device_ControlEndpointSize))
            {
                USB::Endpoint_Write_8(*DataStream);
                DataStream++;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = (BytesInEndpoint == USB_Device_ControlEndpointSize);
            USB::Endpoint_ClearIN();
        }
    }

    while (!(USB::Endpoint_IsOUTReceived()))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t USB::Endpoint_Write_Control_PStream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    bool LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        Endpoint_ClearIN();

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (USB::Endpoint_IsSETUPReceived())
            return ENDPOINT_RWCSTREAM_HostAborted;
        else if (USB::Endpoint_IsOUTReceived())
            break;

        if (USB::Endpoint_IsINReady())
        {
            uint16_t BytesInEndpoint = Endpoint_BytesInEndpoint();

            while (Length && (BytesInEndpoint < USB_Device_ControlEndpointSize))
            {
                Endpoint_Write_8(pgm_read_byte(DataStream));
                DataStream++;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = (BytesInEndpoint == USB_Device_ControlEndpointSize);
            Endpoint_ClearIN();
        }
    }

    while (!(USB::Endpoint_IsOUTReceived()))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

ISR(USB_GEN_vect)
{
    USB *instance = USB::instance;
    instance->gen();
}

void USB::gen()
{
    if (USB_INT_HasOccurred(USB_INT_SOFI) && USB_INT_IsEnabled(USB_INT_SOFI))
    {
        USB_INT_Clear(USB_INT_SOFI);
        //EVENT_USB_Device_StartOfFrame();
    }

    if (USB_INT_HasOccurred(USB_INT_VBUSTI) && USB_INT_IsEnabled(USB_INT_VBUSTI))
    {
        USB_INT_Clear(USB_INT_VBUSTI);

        if (USB_VBUS_GetStatus())
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
            {
                USB_PLL_On();
                while (!(USB_PLL_IsReady()));
            }

            USB_DeviceState = DEVICE_STATE_Powered;
            //EVENT_USB_Device_Connect();
        }
        else
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
                USB_PLL_Off();

            USB_DeviceState = DEVICE_STATE_Unattached;
            //EVENT_USB_Device_Disconnect();
        }
    }

    if (USB_INT_HasOccurred(USB_INT_SUSPI) && USB_INT_IsEnabled(USB_INT_SUSPI))
    {
        USB_INT_Disable(USB_INT_SUSPI);
        USB_INT_Enable(USB_INT_WAKEUPI);
        USB_CLK_Freeze();

        if (!(USB_Options & USB_OPT_MANUAL_PLL))
            USB_PLL_Off();

        USB_DeviceState = DEVICE_STATE_Suspended;
        //EVENT_USB_Device_Suspend();
    }

    if (USB_INT_HasOccurred(USB_INT_WAKEUPI) && USB_INT_IsEnabled(USB_INT_WAKEUPI))
    {
        if (!(USB_Options & USB_OPT_MANUAL_PLL))
        {
            USB_PLL_On();
            while (!(USB_PLL_IsReady()));
        }

        USB_CLK_Unfreeze();
        USB_INT_Clear(USB_INT_WAKEUPI);
        USB_INT_Disable(USB_INT_WAKEUPI);
        USB_INT_Enable(USB_INT_SUSPI);

        if (USB_Device_ConfigurationNumber)
            USB_DeviceState = DEVICE_STATE_Configured;
        else
            USB_DeviceState = (USB_Device_IsAddressSet()) ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

        //EVENT_USB_Device_WakeUp();
    }

    if (USB_INT_HasOccurred(USB_INT_EORSTI) && USB::USB_INT_IsEnabled(USB_INT_EORSTI))
    {
        USB_INT_Clear(USB_INT_EORSTI);
        USB_DeviceState = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        USB_INT_Clear(USB_INT_SUSPI);
        USB_INT_Disable(USB_INT_SUSPI);
        USB_INT_Enable(USB_INT_WAKEUPI);

        Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL,
                            USB_Device_ControlEndpointSize, 1);

        USB_INT_Enable(USB_INT_RXSTPI);
        //EVENT_USB_Device_Reset();
    }
}

ISR(USB_COM_vect)
{
    USB *instance = USB::instance;
    instance->com();
}

void USB::com()
{
    uint8_t PrevSelectedEndpoint = Endpoint_GetCurrentEndpoint();
    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    USB_INT_Disable(USB_INT_RXSTPI);
    GlobalInterruptEnable();
    Device_ProcessControlRequest();
    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    USB_INT_Enable(USB_INT_RXSTPI);
    Endpoint_SelectEndpoint(PrevSelectedEndpoint);
}

bool USB::Endpoint_ConfigureEndpointTable(const USB_Endpoint_Table_t* const Table,
                                     const uint8_t Entries)
{
    for (uint8_t i = 0; i < Entries; i++)
    {
        if (!(Table[i].Address))
            continue;

        if (!(Endpoint_ConfigureEndpoint(Table[i].Address, Table[i].Type,
                    Table[i].Size, Table[i].Banks)))
        {
            return false;
        }
    }

    return true;
}

bool USB::Endpoint_ConfigureEndpoint_Prv(const uint8_t Number, const uint8_t UECFG0XData,
                                    const uint8_t UECFG1XData)
{
    for (uint8_t EPNum = Number; EPNum < ENDPOINT_TOTAL_ENDPOINTS; EPNum++)
    {
        uint8_t UECFG0XTemp;
        uint8_t UECFG1XTemp;
        uint8_t UEIENXTemp;

        Endpoint_SelectEndpoint(EPNum);

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

        if (!(UECFG1XTemp & (1 << ALLOC)))
          continue;

        Endpoint_DisableEndpoint();
        UECFG1X &= ~(1 << ALLOC);
        Endpoint_EnableEndpoint();
        UECFG0X = UECFG0XTemp;
        UECFG1X = UECFG1XTemp;
        UEIENX  = UEIENXTemp;

       if (!(USB::Endpoint_IsConfigured()))
          return false;
    }

    Endpoint_SelectEndpoint(Number);
    return true;
}

uint8_t USB::Endpoint_GetEndpointDirection()
{
    return UECFG0X & 1<<EPDIR ? ENDPOINT_DIR_IN : ENDPOINT_DIR_OUT;
}

void USB::Endpoint_ResetEndpoint(const uint8_t addr)
{
        UERST = 1<<(addr & ENDPOINT_EPNUM_MASK);
        UERST = 0;
}

void USB::Endpoint_ClearStatusStage()
{
    if (USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST)
    {
        while (!(USB::Endpoint_IsOUTReceived()))
        {
            if (USB_DeviceState == DEVICE_STATE_Unattached)
              return;
        }

        Endpoint_ClearOUT();
    }
    else
    {
        while (!(USB::Endpoint_IsINReady()))
        {
            if (USB_DeviceState == DEVICE_STATE_Unattached)
              return;
        }

        Endpoint_ClearIN();
    }
}

uint8_t USB::Endpoint_WaitUntilReady()
{
    uint16_t TimeoutMSRem = USB_STREAM_TIMEOUT_MS;
    uint16_t PreviousFrameNumber = USB_Device_GetFrameNumber();

    while (true)
    {
        if (Endpoint_GetEndpointDirection() == ENDPOINT_DIR_IN)
        {
            if (USB::Endpoint_IsINReady())
                return ENDPOINT_READYWAIT_NoError;
        }
        else
        {
            if (USB::Endpoint_IsOUTReceived())
                return ENDPOINT_READYWAIT_NoError;
        }

        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_READYWAIT_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_READYWAIT_BusSuspended;
        else if (Endpoint_IsStalled())
            return ENDPOINT_READYWAIT_EndpointStalled;

        uint16_t CurrentFrameNumber = USB::USB_Device_GetFrameNumber();

        if (CurrentFrameNumber != PreviousFrameNumber)
        {
            PreviousFrameNumber = CurrentFrameNumber;

            if (!(TimeoutMSRem--))
                return ENDPOINT_READYWAIT_Timeout;
        }
    }
}

uint8_t USB_Device_ConfigurationNumber;
bool USB_Device_CurrentlySelfPowered;
bool USB_Device_RemoteWakeupEnabled;

void USB::Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t RequestHeaderByte = 0; RequestHeaderByte < sizeof(USB_Request_Header_t);
                   RequestHeaderByte++)
    {
        *(RequestHeader++) = USB::Endpoint_Read_8();
    }

    EVENT_USB_Device_ControlRequest();

    if (USB::Endpoint_IsSETUPReceived())
    {
        uint8_t bmRequestType = USB_ControlRequest.bmRequestType;

        switch (USB_ControlRequest.bRequest)
        {
        case REQ_GetStatus:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                Device_GetStatus();
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
                Device_SetAddress();
            break;
        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                Device_GetDescriptor();
            }

            break;
        case REQ_GetConfiguration:
				if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
				  Device_GetConfiguration();

            break;
        case REQ_SetConfiguration:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
                Device_SetConfiguration();
            break;
        default:
            break;
        }
    }

    if (Endpoint_IsSETUPReceived())
    {
        Endpoint_ClearSETUP();
        Endpoint_StallTransaction();
    }
}

void USB::SetGlobalInterruptMask(const uint_reg_t GlobalIntState)
{
    GCC_MEMORY_BARRIER();
    SREG = GlobalIntState;
    GCC_MEMORY_BARRIER();
}

void USB::GlobalInterruptEnable()
{
    GCC_MEMORY_BARRIER();
    sei();
    GCC_MEMORY_BARRIER();
}

void USB::GlobalInterruptDisable()
{
    GCC_MEMORY_BARRIER();
    cli();
    GCC_MEMORY_BARRIER();
}

uint8_t USB::Endpoint_GetCurrentEndpoint()
{
    return UENUM & ENDPOINT_EPNUM_MASK | Endpoint_GetEndpointDirection();
}

void USB::Device_SetAddress()
{
    uint8_t DeviceAddress = (USB_ControlRequest.wValue & 0x7F);
    USB_Device_SetDeviceAddress(DeviceAddress);
    Endpoint_ClearSETUP();
    Endpoint_ClearStatusStage();
    while (!(Endpoint_IsINReady()));
    USB_Device_EnableDeviceAddress(DeviceAddress);
    USB_DeviceState = (DeviceAddress) ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
}

void USB::Device_SetConfiguration()
{
    if ((uint8_t)USB_ControlRequest.wValue > FIXED_NUM_CONFIGURATIONS)
        return;

    Endpoint_ClearSETUP();
    USB_Device_ConfigurationNumber = (uint8_t)USB_ControlRequest.wValue;
    Endpoint_ClearStatusStage();

    if (USB_Device_ConfigurationNumber)
        USB_DeviceState = DEVICE_STATE_Configured;
    else
        USB_DeviceState=(USB_Device_IsAddressSet())?DEVICE_STATE_Configured:DEVICE_STATE_Powered;

    EVENT_USB_Device_ConfigurationChanged();
}

void USB::Device_GetConfiguration()
{
	Endpoint_ClearSETUP();
	Endpoint_Write_8(USB_Device_ConfigurationNumber);
	Endpoint_ClearIN();
	Endpoint_ClearStatusStage();
}

void USB::Device_GetInternalSerialDescriptor()
{
    struct
    {
        USB_Descriptor_Header_t Header;
        uint16_t UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
    }
    SignatureDescriptor;

	SignatureDescriptor.Header.Type = DTYPE_String;
	SignatureDescriptor.Header.Size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
	Device_GetSerialString(SignatureDescriptor.UnicodeString);
	Endpoint_ClearSETUP();
	Endpoint_Write_Control_Stream_LE(&SignatureDescriptor, sizeof(SignatureDescriptor));
	Endpoint_ClearOUT();
}

void USB::Device_GetDescriptor()
{
	const void* DescriptorPointer;
	uint16_t DescriptorSize;

	if (USB_ControlRequest.wValue == ((DTYPE_String << 8) | USE_INTERNAL_SERIAL))
	{
		Device_GetInternalSerialDescriptor();
		return;
	}

    if ((DescriptorSize = CALLBACK_USB_GetDescriptor(USB_ControlRequest.wValue,
                          USB_ControlRequest.wIndex, &DescriptorPointer)) == NO_DESCRIPTOR)
	{
		return;
	}

	Endpoint_ClearSETUP();
    Endpoint_Write_Control_PStream_LE(DescriptorPointer, DescriptorSize);
    Endpoint_ClearOUT();
}

void USB::Device_GetStatus()
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
            Endpoint_SelectEndpoint((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);
            CurrentStatus = USB::Endpoint_IsStalled();
            Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
			break;
        default:
            return;
	}

	Endpoint_ClearSETUP();
	Endpoint_Write_16_LE(CurrentStatus);
	Endpoint_ClearIN();
	Endpoint_ClearStatusStage();
}

uint32_t USB::Endpoint_Read_32_LE()
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

void USB::Device_ClearSetFeature()
{
	switch (USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_RECIPIENT)
	{
    case REQREC_DEVICE:
        if ((uint8_t)USB_ControlRequest.wValue == FEATURE_SEL_DeviceRemoteWakeup)
            USB_Device_RemoteWakeupEnabled = (USB_ControlRequest.bRequest == REQ_SetFeature);
        else
            return;

        break;
    case REQREC_ENDPOINT:
        if ((uint8_t)USB_ControlRequest.wValue == FEATURE_SEL_EndpointHalt)
        {
            uint8_t EndpointIndex = ((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);

            if (EndpointIndex == ENDPOINT_CONTROLEP)
                return;

            Endpoint_SelectEndpoint(EndpointIndex);

            if (Endpoint_IsEnabled())
            {
                if (USB_ControlRequest.bRequest == REQ_SetFeature)
                {
                    Endpoint_StallTransaction();
                }
                else
                {
                    Endpoint_ClearStall();
                    Endpoint_ResetEndpoint(EndpointIndex);
                    Endpoint_ResetDataToggle();
                }
            }
        }

        break;
    default:
        return;
	}

    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    Endpoint_ClearSETUP();
    Endpoint_ClearStatusStage();
}

bool USB::Endpoint_HasEndpointInterrupted(const uint8_t Address)
{
    return ((Endpoint_GetEndpointInterrupts() &
            (1 << (Address & ENDPOINT_EPNUM_MASK))) ? true : false);
}

void USB::Endpoint_SetEndpointDirection(const uint8_t DirectionMask)
{
    UECFG0X = ((UECFG0X & ~(1<<EPDIR)) | (DirectionMask ? (1<<EPDIR) : 0));
}

void Endpoint_Write_32_BE(const uint32_t Data)
{
    UEDATX = Data >> 24;
    UEDATX = Data >> 16;
    UEDATX = Data >> 8;
    UEDATX = Data &  0xFF;
}

void USB::Endpoint_Write_16_LE(const uint16_t Data)
{
    UEDATX = (Data & 0xFF);
    UEDATX = (Data >> 8);
}

void USB::Endpoint_Discard_32()
{
    uint8_t Dummy;
    Dummy = UEDATX;
    Dummy = UEDATX;
    Dummy = UEDATX;
    Dummy = UEDATX;
    (void)Dummy;
}

void USB::USB_INT_Clear(const uint8_t Interrupt)
{
    switch (Interrupt)
    {
    case USB_INT_VBUSTI:
        USBINT &= ~(1<<VBUSTI);
        break;
    case USB_INT_WAKEUPI:
        UDINT &= ~(1<<WAKEUPI);
        break;
    case USB_INT_SUSPI:
        UDINT &= ~(1<<SUSPI);
        break;
    case USB_INT_EORSTI:
        UDINT &= ~(1<<EORSTI);
        break;
    case USB_INT_SOFI:
        UDINT &= ~(1<<SOFI);
        break;
    case USB_INT_RXSTPI:
        UEINTX &= ~(1<<RXSTPI);
        break;
    default:
        break;
    }
}

bool USB::USB_INT_IsEnabled(const uint8_t Interrupt)
{
    switch (Interrupt)
    {
     case USB_INT_VBUSTI:
         return USBCON & 1<<VBUSTE;
     case USB_INT_WAKEUPI:
         return UDIEN & 1<<WAKEUPE;
     case USB_INT_SUSPI:
         return UDIEN & 1<<SUSPE;
     case USB_INT_EORSTI:
         return UDIEN & 1<<EORSTE;
     case USB_INT_SOFI:
         return UDIEN & 1<<SOFE;
     case USB_INT_RXSTPI:
         return UEIENX & 1<<RXSTPE;
    default:
        return false;
    }
}

void USB::Endpoint_Write_32_LE(const uint32_t Data)
{
    UEDATX = Data &  0xFF;
    UEDATX = Data >> 8;
    UEDATX = Data >> 16;
    UEDATX = Data >> 24;
}

void USB::Endpoint_Write_16_BE(const uint16_t Data)
{
    UEDATX = Data>>8;
    UEDATX = Data & 0xFF;
}

void USB::Endpoint_Discard_16()
{
    uint8_t Dummy;
    Dummy = UEDATX;
    Dummy = UEDATX;
    (void)Dummy;
}

void USB::Endpoint_AbortPendingIN()
{
    while (Endpoint_GetBusyBanks() != 0)
    {
        UEINTX |= (1 << RXOUTI);
        while (UEINTX & (1 << RXOUTI));
    }
}

uint16_t USB::Endpoint_Read_16_LE()
{
    union
    {
        uint16_t Value;
        uint8_t  Bytes[2];
    } Data;

    Data.Bytes[0] = UEDATX;
    Data.Bytes[1] = UEDATX;
    return Data.Value;
}

uint16_t USB::Endpoint_Read_16_BE()
{
    union
    {
        uint16_t Value;
        uint8_t  Bytes[2];
    } Data;

    Data.Bytes[1] = UEDATX;
    Data.Bytes[0] = UEDATX;
    return Data.Value;
}

uint32_t USB::Endpoint_Read_32_BE()
{
    union
    {
        uint32_t Value;
        uint8_t  Bytes[4];
    } Data;

    Data.Bytes[3] = UEDATX;
    Data.Bytes[2] = UEDATX;
    Data.Bytes[1] = UEDATX;
    Data.Bytes[0] = UEDATX;
    return Data.Value;
}

void USB::USB_INT_Enable(const uint8_t Interrupt)
{
    switch (Interrupt)
    {
    case USB_INT_VBUSTI:
        USBCON |= 1<<VBUSTE;
        break;
    case USB_INT_WAKEUPI:
        UDIEN  |= 1<<WAKEUPE;
        break;
    case USB_INT_SUSPI:
        UDIEN  |= 1<<SUSPE;
        break;
    case USB_INT_EORSTI:
        UDIEN  |= 1<<EORSTE;
        break;
    case USB_INT_SOFI:
        UDIEN  |= 1<<SOFE;
        break;
    case USB_INT_RXSTPI:
        UEIENX |= 1<<RXSTPE;
        break;
    default:
        break;
    }
}

void USB::USB_INT_Disable(const uint8_t Interrupt)
{
    switch (Interrupt)
    {
    case USB_INT_VBUSTI:
        USBCON &= ~(1<<VBUSTE);
        break;
    case USB_INT_WAKEUPI:
        UDIEN  &= ~(1<<WAKEUPE);
        break;
    case USB_INT_SUSPI:
        UDIEN  &= ~(1<<SUSPE);
        break;
    case USB_INT_EORSTI:
        UDIEN  &= ~(1<<EORSTE);
        break;
    case USB_INT_SOFI:
        UDIEN  &= ~(1<<SOFE);
        break;
    case USB_INT_RXSTPI:
        UEIENX &= ~(1<<RXSTPE);
        break;
    default:
        break;
    }
}

bool USB::USB_INT_HasOccurred(const uint8_t Interrupt)
{
    switch (Interrupt)
    {
    case USB_INT_VBUSTI:
        return USBINT & 1<<VBUSTI;
    case USB_INT_WAKEUPI:
        return UDINT & 1<<WAKEUPI;
    case USB_INT_SUSPI:
        return (UDINT  & (1 << SUSPI));
    case USB_INT_EORSTI:
        return (UDINT  & (1 << EORSTI));
    case USB_INT_SOFI:
        return UDINT  & (1 << SOFI);
    case USB_INT_RXSTPI:
        return UEINTX & 1<<RXSTPI;
    default:
        return false;
    }
}

inline void USB::Device_GetSerialString(uint16_t* const UnicodeString)
{
    uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
    GlobalInterruptDisable();
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

    UnicodeString[SerialCharNum] = cpu_to_le16((SerialByte >= 10) ?
                               (('A' - 10) + SerialByte) : ('0' + SerialByte));
    }

    SetGlobalInterruptMask(CurrentGlobalInt);
}

void USB::Endpoint_Discard_8()
{
    uint8_t Dummy;
    Dummy = UEDATX;
    (void)Dummy;
}





