#ifndef _BUSBY_H_
#define _BUSBY_H_
#include <avr/io.h>
#include <stddef.h>

static constexpr uint8_t
    ENDPOINT_DIR_MASK = 0x80,
    ENDPOINT_DIR_OUT = 0x00,
    ENDPOINT_DIR_IN = 0x80,
    EP_TYPE_MASK = 3,
    EP_TYPE_CONTROL = 0,
    EP_TYPE_ISOCHRONOUS = 1,
    EP_TYPE_BULK = 2,
    EP_TYPE_INTERRUPT = 3,
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
    ENDPOINT_ATTR_NO_SYNC = 0<<2,
    ENDPOINT_ATTR_ASYNC = 1<<2,
    ENDPOINT_ATTR_ADAPTIVE = 2<<2,
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
    REQ_SetAddress = 5,
    REQ_GetDescriptor = 6,
    REQ_SetDescriptor = 7,
    REQ_GetConfiguration = 8,
    REQ_SetConfiguration = 9,
    REQ_GetInterface = 10,
    REQ_SetInterface = 11,
    REQ_SynchFrame = 12,
    ENDPOINT_READYWAIT_NoError = 0,
    ENDPOINT_READYWAIT_EndpointStalled = 1,
    ENDPOINT_READYWAIT_DeviceDisconnected = 2,
    ENDPOINT_READYWAIT_BusSuspended = 3,
    ENDPOINT_READYWAIT_Timeout = 4,
    ENDPOINT_RWCSTREAM_NoError            = 0,
    ENDPOINT_RWCSTREAM_HostAborted        = 1,
    ENDPOINT_RWCSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWCSTREAM_BusSuspended       = 3,
    USB_CSCP_NoDeviceClass = 0,
    USB_CSCP_NoSpecificSubclass = 0,
    USB_CSCP_NoSpecificProtocol = 0,
    USB_CSCP_NoDeviceSubclass = 0,
    USB_CSCP_NoDeviceProtocol = 0,
    DTYPE_Device          = 1,
    DTYPE_Configuration   = 2,
    DTYPE_String          = 3,
    DTYPE_Interface       = 4,
    DTYPE_Endpoint        = 5,
    DTYPE_DeviceQualifier = 6,
    DTYPE_Other           = 7,
    DTYPE_InterfacePower = 0x08,
    DTYPE_InterfaceAssociation = 0x0B,
    DTYPE_CSInterface = 0x24,
    DTYPE_CSEndpoint = 0x25,
    USB_DEVICE_OPT_LOWSPEED = 1<<0,
    FIXED_CONTROL_ENDPOINT_SIZE = 8,
    FIXED_NUM_CONFIGURATIONS = 1,
    NO_DESCRIPTOR = 0;

struct USB_Request_Header_t
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
}
__attribute__ ((packed));


#define GCC_MEMORY_BARRIER()                  __asm__ __volatile__("" ::: "memory");


typedef uint8_t uint_reg_t;

static constexpr uint8_t USB_CONFIG_POWER_MA(uint8_t mA) { return mA >> 1; }

struct DescHeader
{
    uint8_t size;
    uint8_t type;
}
__attribute__ ((packed));

static constexpr size_t USB_STRING_LEN(size_t uniChars)
{ return sizeof(DescHeader) + (uniChars << 1); }

struct DescDev
{
    DescHeader header;
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

struct DescConf
{
    DescHeader header;
    uint16_t TotalConfigurationSize;
    uint8_t TotalInterfaces;
    uint8_t ConfigurationNumber;
    uint8_t ConfigurationStrIndex;
    uint8_t ConfigAttributes;
    uint8_t MaxPowerConsumption;
}
__attribute__ ((packed));

struct DescIface
{
    DescHeader header;
    uint8_t InterfaceNumber;
    uint8_t AlternateSetting;
    uint8_t TotalEndpoints;
    uint8_t Class;
    uint8_t SubClass;
    uint8_t Protocol;
    uint8_t InterfaceStrIndex;
} __attribute__ ((packed));

struct DescEndpoint
{
    DescHeader Header;
    uint8_t EndpointAddress;
    uint8_t Attributes;
    uint16_t EndpointSize;
    uint8_t PollingIntervalMS;
}
__attribute__ ((packed));

template <size_t S> struct USB_Descriptor_String_t
{
    DescHeader Header;
    wchar_t UnicodeString[S];
};

struct Endpoint
{
    const uint8_t addr;
    const uint16_t size;
    const uint8_t type;
    const uint8_t banks;

    void select() { UENUM = addr & ENDPOINT_EPNUM_MASK; }
    void reset() { UERST = 1<<(addr & ENDPOINT_EPNUM_MASK); UERST = 0; }

    Endpoint(uint8_t addr2, uint16_t size2, uint8_t type2, uint8_t banks2) :
        addr(addr2), size(size2), type(type2), banks(banks2) { }
};

class USB
{
protected:
    static const uint8_t
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
        USB_MODE_UID    = 3;
protected:
    USB_Request_Header_t USB_ControlRequest;
    volatile bool USB_IsInitialized;
    volatile uint8_t state;
    uint8_t USB_Device_ConfigurationNumber;
    bool USB_Device_CurrentlySelfPowered;
    bool USB_Device_RemoteWakeupEnabled;
    Endpoint _control;
    uint8_t write_Control_Stream_LE(const void * const buf, uint16_t len);
    uint8_t write_Control_PStream_LE(const void * const buf, uint16_t len);
    uint8_t writeStream(const void * const buf, uint16_t len, uint16_t * const bytes);
    uint8_t readStream(void * const buf, uint16_t len, uint16_t * const bytes);
    uint8_t nullStream(uint16_t len, uint16_t * const bytesProcessed);
    static constexpr uint8_t USB_Options = USB_OPT_REG_ENABLED | USB_OPT_AUTO_PLL;
    uint8_t Endpoint_WaitUntilReady();
    uint8_t getEndpointDirection();
    uint8_t getCurrentEndpoint() { return UENUM & ENDPOINT_EPNUM_MASK | getEndpointDirection(); }
    void selectEndpoint(uint8_t addr) { UENUM = addr & ENDPOINT_EPNUM_MASK; }
    inline uint8_t read8() const { return UEDATX; }
    uint32_t read32();
    inline void write8(uint8_t dat) { UEDATX = dat; }
    inline void write16(uint16_t dat) { UEDATX = dat & 0xff; UEDATX = dat >> 8; }
    void write32(uint32_t dat);
    inline uint16_t bytesInEndpoint() { return ((uint16_t)UEBCHX<<8) | UEBCLX; }
    void Device_ClearSetFeature();
    void Device_GetSerialString(uint16_t *UnicodeString);
    void Endpoint_ClearStatusStage();
    bool configureEndpoint(uint8_t addr, uint8_t type, uint16_t size, uint8_t banks);
    bool configureEndpoint(Endpoint &ep);
    uint8_t Endpoint_BytesToEPSizeMask(const uint16_t Bytes);
public:
    static USB *instance;
    USB();
    virtual void gen() { }
    virtual void com() { }
};

#endif



