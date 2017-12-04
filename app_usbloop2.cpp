#define F_CPU 16000000UL

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/power.h>



static constexpr uint8_t ENDPOINT_DIR_MASK = 0x80,
    ENDPOINT_DIR_OUT = 0x00,
    ENDPOINT_DIR_IN = 0x80,
    PIPE_DIR_MASK = 0x80,
    PIPE_DIR_OUT = 0x00,
    PIPE_DIR_IN = 0x80,
    EP_TYPE_MASK = 0x03,
    EP_TYPE_CONTROL = 0x00,
    EP_TYPE_ISOCHRONOUS = 0x01,
    EP_TYPE_BULK = 0x02,
    EP_TYPE_INTERRUPT = 0x03,
    USB_PLL_PSC = 1<<PINDIV,       // for 16MHz; choose 0 for 8MHz
    USB_OPT_REG_DISABLED = 1<<1,
    USB_OPT_REG_ENABLED = 0<<1,
    USB_OPT_MANUAL_PLL = 1<<2,
    USB_OPT_AUTO_PLL = 0<<2,
    USB_STREAM_TIMEOUT_MS = 100,
    ENDPOINT_EPNUM_MASK = 0x0f,
    ENDPOINT_CONTROLEP = 0,
    ENDPOINT_TOTAL_ENDPOINTS = 7,
    USB_CONFIG_ATTR_RESERVED = 0x80,
    USB_CONFIG_ATTR_SELFPOWERED = 0x40,
    USB_CONFIG_ATTR_REMOTEWAKEUP = 0x20,
    ENDPOINT_ATTR_NO_SYNC = 0 << 2,
    ENDPOINT_ATTR_ASYNC = 1 << 2,
    ENDPOINT_ATTR_ADAPTIVE = 2 << 2,
    ENDPOINT_ATTR_SYNC = 3<<2,
    ENDPOINT_USAGE_DATA = 0<<4,
    ENDPOINT_USAGE_FEEDBACK = 1<<4,
    ENDPOINT_USAGE_IMPLICIT_FEEDBACK = 2<<4,
    CONTROL_REQTYPE_DIRECTION = 0x80,
    CONTROL_REQTYPE_TYPE = 0x60,
    CONTROL_REQTYPE_RECIPIENT = 0x1f,
    REQDIR_HOSTTODEVICE = 0<<7,
    REQDIR_DEVICETOHOST = 1<<7,
    REQTYPE_STANDARD = 0<<5,
    REQTYPE_CLASS = 1<<5,
    REQTYPE_VENDOR = 2<<5,
    REQREC_DEVICE = 0<<0,
    REQREC_INTERFACE = 1<<0,
    REQREC_ENDPOINT = 2<<0,
    REQREC_OTHER = 3<<0,
    FEATURE_SELFPOWERED_ENABLED = 1<<0,
    FEATURE_REMOTE_WAKEUP_ENABLED = 1<<1,
    ENDPOINT_RWSTREAM_NoError            = 0,
    ENDPOINT_RWSTREAM_EndpointStalled    = 1,
    ENDPOINT_RWSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWSTREAM_BusSuspended       = 3,
    ENDPOINT_RWSTREAM_Timeout            = 4,
    ENDPOINT_RWSTREAM_IncompleteTransfer = 5,
    USB_DEVICE_OPT_FULLSPEED = 0<<0,
    USE_INTERNAL_SERIAL = 0xDC,
    INTERNAL_SERIAL_LENGTH_BITS = 80,
    INTERNAL_SERIAL_START_ADDRESS = 0x0e,
    REQ_GetStatus = 0,
    REQ_ClearFeature = 1,
    REQ_SetFeature = 3,
    REQ_SetAddress          = 5,
    REQ_GetDescriptor = 6,
    REQ_SetDescriptor       = 7,
    REQ_GetConfiguration    = 8,
    REQ_SetConfiguration    = 9,
    REQ_GetInterface        = 10,
    REQ_SetInterface        = 11,
    REQ_SynchFrame          = 12,
    ENDPOINT_READYWAIT_NoError = 0,
    ENDPOINT_READYWAIT_EndpointStalled = 1,
    ENDPOINT_READYWAIT_DeviceDisconnected = 2,
    ENDPOINT_READYWAIT_BusSuspended = 3,
    ENDPOINT_READYWAIT_Timeout = 4,
    ENDPOINT_RWCSTREAM_NoError            = 0,
    ENDPOINT_RWCSTREAM_HostAborted        = 1,
    ENDPOINT_RWCSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWCSTREAM_BusSuspended       = 3,
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
    CDC_CSCP_CDCClass = 0x02,
    CDC_CSCP_NoSpecificSubclass = 0x00,
    CDC_CSCP_ACMSubclass = 0x02,
    CDC_CSCP_ATCommandProtocol = 0x01,
    CDC_CSCP_NoSpecificProtocol = 0x00,
    CDC_CSCP_VendorSpecificProtocol = 0xFF,
    CDC_CSCP_CDCDataClass = 0x0A,
    CDC_CSCP_NoDataSubclass = 0x00,
    CDC_CSCP_NoDataProtocol = 0x00,
    CDC_REQ_SendEncapsulatedCommand = 0x00,
    CDC_REQ_GetEncapsulatedResponse = 0x01,
    CDC_REQ_SetLineEncoding = 0x20,
    CDC_REQ_GetLineEncoding = 0x21,
    CDC_REQ_SetControlLineState = 0x22,
    CDC_REQ_SendBreak = 0x23,
    CDC_NOTIF_SerialState = 0x20,
    CDC_LINEENCODING_OneStopBit          = 0,
    CDC_LINEENCODING_OneAndAHalfStopBits = 1,
    CDC_LINEENCODING_TwoStopBits         = 2,
    CDC_PARITY_None  = 0,
    CDC_PARITY_Odd   = 1,
    CDC_PARITY_Even  = 2,
    CDC_PARITY_Mark  = 3,
    CDC_PARITY_Space = 4,
    FEATURE_SEL_EndpointHalt       = 0x00,
    FEATURE_SEL_DeviceRemoteWakeup = 0x01,
    FEATURE_SEL_TestMode = 0x02,
    DEVICE_STATE_Unattached  = 0,
    DEVICE_STATE_Powered = 1,
    DEVICE_STATE_Default = 2,
    DEVICE_STATE_Addressed = 3,
    DEVICE_STATE_Configured = 4,
    DEVICE_STATE_Suspended = 5,
    USB_MODE_None   = 0,
    USB_MODE_Device = 1,
    USB_MODE_Host   = 2,
    USB_MODE_UID    = 3,
    DTYPE_Device                    = 0x01,
    DTYPE_Configuration             = 0x02,
    DTYPE_String                    = 0x03,
    DTYPE_Interface                 = 0x04,
    DTYPE_Endpoint                  = 0x05,
    DTYPE_DeviceQualifier           = 0x06,
    DTYPE_Other                     = 0x07,
    DTYPE_InterfacePower            = 0x08,
    DTYPE_InterfaceAssociation      = 0x0B,
    DTYPE_CSInterface               = 0x24,
    DTYPE_CSEndpoint                = 0x25,
    USB_DEVICE_OPT_LOWSPEED = 1<<0;

void USB_Device_ProcessControlRequest();
uint8_t USB_Device_ConfigurationNumber;
bool USB_Device_CurrentlySelfPowered;
bool USB_Device_RemoteWakeupEnabled;

struct USB_Request_Header_t
{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__ ((packed));

#define GCC_MEMORY_BARRIER()                  __asm__ __volatile__("" ::: "memory");

#define ATTR_WARN_UNUSED_RESULT      __attribute__ ((warn_unused_result))
#define ATTR_ALWAYS_INLINE           __attribute__ ((always_inline))
#define ATTR_PACKED                      __attribute__ ((packed))

static const uint8_t FIXED_CONTROL_ENDPOINT_SIZE = 8;
static const uint8_t  USB_Device_ControlEndpointSize = FIXED_CONTROL_ENDPOINT_SIZE;
static const uint8_t FIXED_NUM_CONFIGURATIONS = 1;

typedef uint8_t uint_reg_t;

static constexpr uint32_t LE32_TO_CPU(uint32_t x) { return x; }
static constexpr uint16_t CPU_TO_LE16(uint16_t x) { return x; }

static const uint8_t NO_DESCRIPTOR = 0;
#define USB_CONFIG_POWER_MA(mA)           ((mA) >> 1)
#define USB_STRING_LEN(UnicodeChars) (sizeof(USB_Descriptor_Header_t) + ((UnicodeChars) << 1))

#define VERSION_BCD(x)   CPU_TO_LE16((VERSION_TENS(x) << 12)  | (VERSION_ONES(x) << 8) | \
                              (VERSION_TENTHS(x) << 4) | (VERSION_HUNDREDTHS(x) << 0) )


typedef struct
{
    uint8_t Size;
    uint8_t Type;
} ATTR_PACKED USB_Descriptor_Header_t;

struct USB_Descriptor_Device_t
{
    USB_Descriptor_Header_t Header;
    uint16_t USBSpecification;
    uint8_t  Class;
    uint8_t  SubClass;
    uint8_t  Protocol;
    uint8_t  Endpoint0Size;
    uint16_t VendorID;
    uint16_t ProductID;
    uint16_t ReleaseNumber;
    uint8_t  ManufacturerStrIndex;
    uint8_t  ProductStrIndex;
    uint8_t  SerialNumStrIndex;
    uint8_t  NumberOfConfigurations;
} __attribute__ ((packed));

struct USB_Descriptor_DeviceQualifier_t
{
    USB_Descriptor_Header_t Header;
    uint16_t USBSpecification;
    uint8_t Class;
    uint8_t SubClass;
    uint8_t Protocol;
    uint8_t Endpoint0Size;
    uint8_t NumberOfConfigurations;
    uint8_t Reserved;
} __attribute__ ((packed));

typedef struct
{
    USB_Descriptor_Header_t Header;
    uint16_t TotalConfigurationSize;
    uint8_t TotalInterfaces;
    uint8_t ConfigurationNumber;
    uint8_t ConfigurationStrIndex;
    uint8_t ConfigAttributes;
    uint8_t MaxPowerConsumption;
} ATTR_PACKED USB_Descriptor_Configuration_Header_t;

typedef struct
{
    USB_Descriptor_Header_t Header;
    uint8_t InterfaceNumber;
    uint8_t AlternateSetting;
    uint8_t TotalEndpoints;
    uint8_t Class;
    uint8_t SubClass;
    uint8_t Protocol;
    uint8_t InterfaceStrIndex;
} ATTR_PACKED USB_Descriptor_Interface_t;

struct USB_Descriptor_Interface_Association_t
{
    USB_Descriptor_Header_t Header;
    uint8_t FirstInterfaceIndex;
    uint8_t TotalInterfaces;
    uint8_t Class;
    uint8_t SubClass;
    uint8_t Protocol;
    uint8_t IADStrIndex;
} __attribute__ ((packed));

typedef struct
{
    USB_Descriptor_Header_t Header;
    uint8_t EndpointAddress;
    uint8_t Attributes;
    uint16_t EndpointSize;
    uint8_t PollingIntervalMS;
} ATTR_PACKED USB_Descriptor_Endpoint_t;

typedef struct
{
    USB_Descriptor_Header_t Header;
    wchar_t UnicodeString[50];
} ATTR_PACKED USB_Descriptor_String_t;

#define VERSION_TENS(x)                   (int)((int)(x) / 10)
#define VERSION_ONES(x)                   (int)((int)(x) % 10)
#define VERSION_TENTHS(x)                 (int)((x - (int)x) * 10)
#define VERSION_HUNDREDTHS(x)             (int)((x * 100) - ((int)(x * 10) * 10))

struct Endpoint
{
    const uint8_t addr;
    const uint16_t Size;
    const uint8_t  Type;
    const uint8_t  Banks;

    void select() { UENUM = addr & ENDPOINT_EPNUM_MASK; }

    Endpoint(uint8_t addr2, uint16_t size2, uint8_t type2, uint8_t banks2) :
        addr(addr2), Size(size2), Type(type2), Banks(banks2) { }
};

struct CDC_LineEncoding_t
{
    uint32_t BaudRateBPS;
    uint8_t CharFormat;
    uint8_t ParityType;
    uint8_t DataBits;
};

class USB
{
private:
    volatile bool USB_IsInitialized;
    USB_Request_Header_t USB_ControlRequest;
    Endpoint _control;
    Endpoint _inpoint;
    Endpoint _outpoint;
    Endpoint _notif;
    CDC_LineEncoding_t _lineEncoding;
    void Endpoint_ClearStatusStage();
    inline uint8_t Endpoint_Read_8() { return UEDATX; }
    inline void Endpoint_Write_8(const uint8_t Data) { UEDATX = Data; }
    inline void Endpoint_Write_16_LE(const uint16_t Data);
    inline uint32_t Endpoint_Read_32_LE();
    inline void Endpoint_Write_32_LE(const uint32_t Data);
    inline void USB_INT_Clear(const uint8_t Interrupt);
    inline uint16_t Endpoint_BytesInEndpoint() { return ((uint16_t)UEBCHX<<8) | UEBCLX; }
    inline void Device_GetSerialString(uint16_t* const UnicodeString);
    uint8_t Endpoint_GetEndpointDirection();
    void Endpoint_SelectEndpoint(uint8_t addr) { UENUM = addr & ENDPOINT_EPNUM_MASK; }
    uint8_t Endpoint_GetCurrentEndpoint();
    bool CDC_Device_ConfigureEndpoints();
    uint8_t Endpoint_Write_Control_PStream_LE(const void *Buffer, uint16_t len);
    uint8_t Endpoint_WaitUntilReady();
    void EVENT_USB_Device_ControlRequest();
    void Device_ClearSetFeature();
    uint8_t Endpoint_Write_Control_Stream_LE(const void *buffer, uint16_t length);
    void Device_ProcessControlRequest();
    uint8_t Endpoint_BytesToEPSizeMask(const uint16_t Bytes);
    bool Endpoint_ConfigureEndpointTable(Endpoint *table, uint8_t entries);
    bool Endpoint_ConfigureEndpoint(uint8_t addr, uint8_t type, uint16_t size, uint8_t banks);
public:
    int16_t receive();
    uint8_t sendByte(uint8_t data);
    USB();
    uint8_t flush();
    void gen();
    void com();
    static USB *instance;
private:
    static constexpr uint8_t USB_Options = USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED |
            USB_OPT_AUTO_PLL;

    static uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue, const uint8_t wIndex,
                                    const void** const DescriptorAddress);
};

USB *USB::instance;

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

bool USB::CDC_Device_ConfigureEndpoints()
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

uint8_t USB::sendByte(uint8_t Data)
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

    Endpoint_Write_8(Data);
    return ENDPOINT_READYWAIT_NoError;
}

uint8_t USB::flush()
{
    if ((GPIOR0 != DEVICE_STATE_Configured) || !_lineEncoding.BaudRateBPS)
        return ENDPOINT_RWSTREAM_DeviceDisconnected;

    uint8_t ErrorCode;
    _inpoint.select();

    if (!(Endpoint_BytesInEndpoint()))
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

int16_t USB::receive()
{
    if ((GPIOR0 != DEVICE_STATE_Configured) || !(_lineEncoding.BaudRateBPS))
        return -1;

    int16_t ReceivedByte = -1;
    _outpoint.select();

    if (UEINTX & 1<<RXOUTI) // is OUT received?
    {
        if (Endpoint_BytesInEndpoint())
            ReceivedByte = Endpoint_Read_8();

        if (!(Endpoint_BytesInEndpoint()))
            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }

    return ReceivedByte;
}

USB::USB() :
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

    uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();
    _control.select();
    //Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);

    if (UEINTX & 1<<RXSTPI)
        Device_ProcessControlRequest();

    Endpoint_SelectEndpoint(PrevEndpoint);
}

void USB::EVENT_USB_Device_ControlRequest()
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
                Endpoint_Write_32_LE(_lineEncoding.BaudRateBPS);
                Endpoint_Write_8(_lineEncoding.CharFormat);
                Endpoint_Write_8(_lineEncoding.ParityType);
                Endpoint_Write_8(_lineEncoding.DataBits);
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

                _lineEncoding.BaudRateBPS = Endpoint_Read_32_LE();
                _lineEncoding.CharFormat = Endpoint_Read_8();
                _lineEncoding.ParityType = Endpoint_Read_8();
                _lineEncoding.DataBits = Endpoint_Read_8();
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
    {
        .Size = USB_STRING_LEN(11), .Type = DTYPE_String
    },
    //L"Dean Camera"
};

const USB_Descriptor_String_t PROGMEM ProductString =
{
    {
        USB_STRING_LEN(22),
        DTYPE_String
    },
    L"LUFA USB-RS232 Adapter"
};

bool USB::Endpoint_ConfigureEndpoint(const uint8_t Address,
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
        Endpoint_SelectEndpoint(EPNum);

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

    Endpoint_SelectEndpoint(Number);
    return true;
}

uint16_t USB::CALLBACK_USB_GetDescriptor(const uint16_t wValue,
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

uint8_t USB::Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length)
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
            uint16_t BytesInEndpoint = Endpoint_BytesInEndpoint();

            while (Length && (BytesInEndpoint < USB_Device_ControlEndpointSize))
            {
                Endpoint_Write_8(*DataStream);
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

uint8_t USB::Endpoint_Write_Control_PStream_LE(const void* const Buffer, uint16_t Length)
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
            uint16_t BytesInEndpoint = Endpoint_BytesInEndpoint();

            while (Length && (BytesInEndpoint < USB_Device_ControlEndpointSize))
            {
                Endpoint_Write_8(pgm_read_byte(DataStream));
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
    USB *instance = USB::instance;
    instance->gen();
}

void USB::gen()
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
    USB *instance = USB::instance;
    instance->com();
}

void USB::com()
{
    uint8_t PrevSelectedEndpoint = Endpoint_GetCurrentEndpoint();
    _control.select();
    UEIENX &= ~(1<<RXSTPE);
    sei();
    Device_ProcessControlRequest();
    _control.select();
    UEIENX |= 1<<RXSTPE;
    Endpoint_SelectEndpoint(PrevSelectedEndpoint);
}

bool USB::Endpoint_ConfigureEndpointTable(Endpoint *table, uint8_t entries)
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

uint8_t USB::Endpoint_GetEndpointDirection()
{
    return UECFG0X & 1<<EPDIR ? ENDPOINT_DIR_IN : ENDPOINT_DIR_OUT;
}

void USB::Endpoint_ClearStatusStage()
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

uint8_t USB::Endpoint_WaitUntilReady()
{
    uint16_t TimeoutMSRem = USB_STREAM_TIMEOUT_MS;
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

void USB::Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t RequestHeaderByte = 0; RequestHeaderByte < sizeof(USB_Request_Header_t);
                   RequestHeaderByte++)
    {
        *(RequestHeader++) = Endpoint_Read_8();
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
                        Endpoint_SelectEndpoint((uint8_t)USB_ControlRequest.wIndex &
                            ENDPOINT_EPNUM_MASK);

                        CurrentStatus = UECONX & 1<<STALLRQ;
                        Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
            			break;
                    default:
                        return;
            	}
            
                UEINTX &= ~(1<<RXSTPI);
            	Endpoint_Write_16_LE(CurrentStatus);
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

                if (USB_ControlRequest.wValue == ((DTYPE_String << 8) | USE_INTERNAL_SERIAL))
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
                Endpoint_Write_8(USB_Device_ConfigurationNumber);
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

uint8_t USB::Endpoint_GetCurrentEndpoint()
{
    return UENUM & ENDPOINT_EPNUM_MASK | Endpoint_GetEndpointDirection();
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

            Endpoint_SelectEndpoint(EndpointIndex);

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

    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    UEINTX &= ~(1<<RXSTPI);
    Endpoint_ClearStatusStage();
}

void USB::Endpoint_Write_16_LE(const uint16_t Data)
{
    UEDATX = Data & 0xFF;
    UEDATX = Data >> 8;
}

void USB::Endpoint_Write_32_LE(const uint32_t Data)
{
    UEDATX = Data &  0xFF;
    UEDATX = Data >> 8;
    UEDATX = Data >> 16;
    UEDATX = Data >> 24;
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

inline void USB::Device_GetSerialString(uint16_t* const UnicodeString)
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

inline bool isUpper(char c) { return c >= 'A' && c <= 'Z'; }
inline bool isLower(char c) { return c >= 'a' && c <= 'z'; }

inline char convert(char c)
{
    if (isUpper(c)) return c + 32;
    if (isLower(c)) return c - 32;
    return c;
}

inline char nibble(uint8_t n)
{
    return n <= 9 ? '0' + n : 'A' + n - 10;
}

int main()
{
    USB usb;

    while (true)
    {
        uint8_t byte = (uint8_t)usb.receive();

        if (byte != 255)
        {
            usb.sendByte(convert(byte));
            usb.flush();
        }
    }

    return 0;
}


