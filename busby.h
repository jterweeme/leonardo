#ifndef _BUSBY_H_
#define _BUSBY_H_
#include <avr/io.h>
#include <stddef.h>
#include "leonardo.h"

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
    DTYPE_DEVICE = 1,
    DTYPE_CONFIGURATION = 2,
    DTYPE_STRING = 3,
    DTYPE_Interface = 4,
    DTYPE_Endpoint = 5,
    DTYPE_DeviceQualifier = 6,
    DTYPE_CSInterface = 0x24,
    DTYPE_CSEndpoint = 0x25,
    FIXED_CONTROL_ENDPOINT_SIZE = 8,
    FIXED_NUM_CONFIGURATIONS = 1,
    NO_DESCRIPTOR = 0,
    FEATURE_SEL_EndpointHalt = 0x00,
    FEATURE_SEL_DeviceRemoteWakeup = 0x01,
    FEATURE_SEL_TestMode = 0x02,
    DEVICE_STATE_Unattached = 0,
    DEVICE_STATE_Powered = 1,
    DEVICE_STATE_Default = 2,
    DEVICE_STATE_Addressed = 3,
    DEVICE_STATE_Configured = 4,
    DEVICE_STATE_Suspended = 5,
    USE_INTERNAL_SERIAL = 0xDC,
    INTERNAL_SERIAL_LENGTH_BITS = 80,
    INTERNAL_SERIAL_START_ADDRESS = 0x0e,
    UZE_INTERNAL_SERIAL = 0xdc,
    INTERNAL_ZERIAL_START_ADDRESS = 0x0e,
    INTERNAL_ZERIAL_LENGTH_BITS = 80;

struct USBRequest
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
}
__attribute__ ((packed));

#define GCC_MEMORY_BARRIER()                  __asm__ __volatile__("" ::: "memory");

static constexpr uint8_t USB_CONFIG_POWER_MA(uint8_t mA) { return mA >> 1; }

struct SigDesc
{
    uint8_t size;
    uint8_t type;
    uint16_t unicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
};

struct DescDev
{
    uint8_t size;
    uint8_t type;
    uint16_t USBSpecification;
    uint8_t Class;
    uint8_t SubClass;
    uint8_t Protocol;
    uint8_t Endpoint0Size;
    uint16_t VendorID;
    uint16_t ProductID;
    uint16_t ReleaseNumber;
    uint8_t ManufacturerStrIndex;
    uint8_t ProductStrIndex;
    uint8_t SerialNumStrIndex;
    uint8_t NumberOfConfigurations;
} __attribute__ ((packed));

struct DescConf
{
    uint8_t size;
    uint8_t type;
    uint16_t totalConfigurationSize;
    uint8_t totalInterfaces;
    uint8_t configurationNumber;
    uint8_t configurationStrIndex;
    uint8_t configAttributes;
    uint8_t maxPowerConsumption;
}
__attribute__ ((packed));

static constexpr size_t USB_STRING_LEN(size_t uniChars) { return 2 + (uniChars << 1); }

struct DescIface
{
    uint8_t size;
    uint8_t type;
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
    uint8_t size;
    uint8_t type;
    uint8_t endpointAddr;
    uint8_t attributes;
    uint16_t endpointSize;
    uint8_t pollingIntervalMS;
}
__attribute__ ((packed));

template <size_t S> struct DescString
{
    uint8_t size;
    uint8_t type;
    wchar_t UnicodeString[S];
};

struct Endpoint
{
    const uint8_t addr;
    const uint16_t size;
    const uint8_t type;
    const uint8_t banks;

    void select() { *p_uenum = addr & ENDPOINT_EPNUM_MASK; }
    void reset() { *p_uerst = 1<<(addr & ENDPOINT_EPNUM_MASK); *p_uerst = 0; }

    Endpoint(uint8_t addr2, uint16_t size2, uint8_t type2, uint8_t banks2) :
        addr(addr2), size(size2), type(type2), banks(banks2) { }
};

class USB
{
private:
    uint8_t getEndpointDirection() const;
    uint8_t Endpoint_BytesToEPSizeMask(uint16_t bytes) const;
protected:
    USBRequest _ctrlReq;
    volatile uint8_t state;
    uint8_t USB_Device_ConfigurationNumber;
    bool USB_Device_CurrentlySelfPowered;
    bool USB_Device_RemoteWakeupEnabled;
    Endpoint _control;
    uint8_t readControlStreamLE(void * const buf, uint16_t len);
    uint8_t write_Control_Stream_LE(const void * const buf, uint16_t len);
    uint8_t write_Control_PStream_LE(const void * const buf, uint16_t len);
    uint8_t writeStream2(const void * const buf, uint16_t len, uint16_t * const bytes);
    uint8_t readStream(void * const buf, uint16_t len, uint16_t * const bytes);
    uint8_t nullStream(uint16_t len, uint16_t * const bytesProcessed);
    uint8_t waitUntilReady() const;
    inline void selectEndpoint(uint8_t addr) const { *p_uenum = addr & ENDPOINT_EPNUM_MASK; }
    inline uint8_t read8() const { return *p_uedatx; }
    uint32_t read32() const;
    inline void write8(uint8_t dat) const { *p_uedatx = dat; }
    inline void write16(uint16_t dat) const { *p_uedatx = dat & 0xff; *p_uedatx = dat >> 8; }
    inline void write16le(uint16_t dat) const { *p_uedatx = dat & 0xff; *p_uedatx = dat >> 8; }
    void write32(uint32_t dat) const;
    void write32be(uint32_t dat) const;
    inline uint16_t bytesInEndpoint() const { return ((uint16_t)UEBCHX<<8) | UEBCLX; }
    void Device_ClearSetFeature();
    void Device_GetSerialString(uint16_t *unicodeString);
    void clearStatusStage() const;
    bool configureEndpoint(uint8_t addr, uint8_t type, uint16_t size, uint8_t banks);
    bool configureEndpoint(Endpoint &ep);
    virtual uint16_t getDesc(uint16_t, uint16_t, const void ** const) { return 0; }
    virtual void procCtrlReq();
    virtual void connect() { }
    virtual void configure() { }
    virtual void customCtrl() { }

    inline void setDevAddr(uint8_t addr) const
    { *p_udaddr = (*p_udaddr & 1<<adden) | (addr & 0x7f); }

    inline uint16_t read16le() const
    {
        union { uint16_t Value; uint8_t Bytes[2]; } Data;
        Data.Bytes[0] = UEDATX;
        Data.Bytes[1] = UEDATX;
        return Data.Value;
    }

    inline uint8_t getCurrentEndpoint() const
    { return (*p_uenum & ENDPOINT_EPNUM_MASK) | getEndpointDirection(); }
public:
    static USB *instance;
    USB();
    void gen();
    void com();
};

#endif



