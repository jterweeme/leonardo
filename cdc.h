#ifndef _BUSBY_H_
#define _BUSBY_H_

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


struct USB_Descriptor_Header_t
{
    uint8_t Size;
    uint8_t Type;
}
ATTR_PACKED;

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

struct USB_Descriptor_Configuration_Header_t
{
    USB_Descriptor_Header_t Header;
    uint16_t TotalConfigurationSize;
    uint8_t TotalInterfaces;
    uint8_t ConfigurationNumber;
    uint8_t ConfigurationStrIndex;
    uint8_t ConfigAttributes;
    uint8_t MaxPowerConsumption;
}
ATTR_PACKED;

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

struct USB_Descriptor_Endpoint_t
{
    USB_Descriptor_Header_t Header;
    uint8_t EndpointAddress;
    uint8_t Attributes;
    uint16_t EndpointSize;
    uint8_t PollingIntervalMS;
} ATTR_PACKED;

template <size_t S> struct USB_Descriptor_String_t
{
    USB_Descriptor_Header_t Header;
    wchar_t UnicodeString[S];
};

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

class CDC
{
private:
    uint8_t USB_Device_ConfigurationNumber;
    bool USB_Device_CurrentlySelfPowered;
    bool USB_Device_RemoteWakeupEnabled;
    volatile bool USB_IsInitialized;
    USB_Request_Header_t USB_ControlRequest;
    Endpoint _control;
    Endpoint _inpoint;
    Endpoint _outpoint;
    Endpoint _notif;
    CDC_LineEncoding_t _lineEncoding;
    void Endpoint_ClearStatusStage();
    inline uint8_t read8() { return UEDATX; }
    uint32_t read32();
    inline void write8(uint8_t dat) { UEDATX = dat; }
    inline void write16(uint16_t dat) { UEDATX = dat & 0xff; UEDATX = dat >> 8; }
    inline void write32(uint32_t dat);
    inline void USB_INT_Clear(const uint8_t Interrupt);
    inline uint16_t bytesInEndpoint() { return ((uint16_t)UEBCHX<<8) | UEBCLX; }
    inline void Device_GetSerialString(uint16_t* const UnicodeString);
    uint8_t getEndpointDirection();
    void selectEndpoint(uint8_t addr) { UENUM = addr & ENDPOINT_EPNUM_MASK; }
    uint8_t getCurrentEndpoint() { return UENUM & ENDPOINT_EPNUM_MASK | getEndpointDirection(); }
    bool CDC_Device_ConfigureEndpoints();
    void EVENT_USB_Device_ControlRequest();
    uint8_t Endpoint_Write_Control_PStream_LE(const void *buf, uint16_t len);
    uint8_t Endpoint_WaitUntilReady();
    void Device_ClearSetFeature();
    uint8_t Endpoint_Write_Control_Stream_LE(const void *buf, uint16_t len);
    void Device_ProcessControlRequest();
    uint8_t Endpoint_BytesToEPSizeMask(const uint16_t Bytes);
    bool Endpoint_ConfigureEndpointTable(Endpoint *table, uint8_t entries);
    bool Endpoint_ConfigureEndpoint(uint8_t addr, uint8_t type, uint16_t size, uint8_t banks);
public:
    int16_t receive();
    uint8_t sendByte(uint8_t data);
    CDC();
    uint8_t flush();
    void gen();
    void com();
    static CDC *instance;
private:
    static constexpr uint8_t USB_Options = USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED |
            USB_OPT_AUTO_PLL;
    static uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue, const uint8_t wIndex,
                                    const void** const DescriptorAddress);
};

struct USB_CDC_Descriptor_FunctionalHeader_t
{
    USB_Descriptor_Header_t Header;
    uint8_t Subtype;
    uint16_t CDCSpecification;
}
ATTR_PACKED;

struct USB_CDC_StdDescriptor_FunctionalHeader_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint16_t bcdCDC;
}
ATTR_PACKED;

struct USB_CDC_Descriptor_FunctionalACM_t
{
    USB_Descriptor_Header_t Header;
    uint8_t Subtype;
    uint8_t Capabilities;
}
ATTR_PACKED;

struct USB_CDC_StdDescriptor_FunctionalACM_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bmCapabilities;
}
ATTR_PACKED;

struct USB_CDC_Descriptor_FunctionalUnion_t
{
    USB_Descriptor_Header_t Header;
    uint8_t                 Subtype;
    uint8_t                 MasterInterfaceNumber;
    uint8_t                 SlaveInterfaceNumber;
}
ATTR_PACKED;

struct USB_CDC_StdDescriptor_FunctionalUnion_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bMasterInterface;
    uint8_t bSlaveInterface0;
}
ATTR_PACKED;

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


#endif



