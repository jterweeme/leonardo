#ifndef _BUSBY_H_
#define _BUSBY_H_

#define F_CPU 16000000UL

#define USE_LUFA_CONFIG_HEADER
#define USB_SERIES_4_AVR

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <util/delay.h>
#include <stdio.h>
#include "misc.h"

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
    USB_STREAM_TIMEOUT_MS = 100;

enum USB_Interrupts_t
{
    USB_INT_VBUSTI = 0,
    USB_INT_WAKEUPI = 2,
    USB_INT_SUSPI = 3,
    USB_INT_EORSTI = 4,
    USB_INT_SOFI = 5,
    USB_INT_RXSTPI = 6,
};

extern uint8_t USB_Device_ConfigurationNumber;
extern bool USB_Device_RemoteWakeupEnabled;
extern bool USB_Device_CurrentlySelfPowered;
void USB_Device_ProcessControlRequest();

enum CDC_Descriptor_ClassSubclassProtocol_t
{
    CDC_CSCP_CDCClass = 0x02,
    CDC_CSCP_NoSpecificSubclass = 0x00,
    CDC_CSCP_ACMSubclass = 0x02,
    CDC_CSCP_ATCommandProtocol = 0x01,
    CDC_CSCP_NoSpecificProtocol = 0x00,
    CDC_CSCP_VendorSpecificProtocol = 0xFF,
    CDC_CSCP_CDCDataClass = 0x0A,
    CDC_CSCP_NoDataSubclass = 0x00,
    CDC_CSCP_NoDataProtocol = 0x00,
};

enum CDC_ClassRequests_t
{
    CDC_REQ_SendEncapsulatedCommand = 0x00,
    CDC_REQ_GetEncapsulatedResponse = 0x01,
    CDC_REQ_SetLineEncoding = 0x20,
    CDC_REQ_GetLineEncoding = 0x21,
    CDC_REQ_SetControlLineState = 0x22,
    CDC_REQ_SendBreak = 0x23,
};

enum CDC_ClassNotifications_t
{
    CDC_NOTIF_SerialState = 0x20,
};

enum CDC_DescriptorSubtypes_t
{
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
};

enum CDC_LineEncodingFormats_t
{
    CDC_LINEENCODING_OneStopBit          = 0,
    CDC_LINEENCODING_OneAndAHalfStopBits = 1,
    CDC_LINEENCODING_TwoStopBits         = 2,
};

enum CDC_LineEncodingParity_t
{
    CDC_PARITY_None  = 0,
    CDC_PARITY_Odd   = 1,
    CDC_PARITY_Even  = 2,
    CDC_PARITY_Mark  = 3,
    CDC_PARITY_Space = 4,
};

struct USB_Request_Header_t
{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__ ((packed));

enum USB_Control_Request_t
{
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
};

enum USB_Feature_Selectors_t
{
    FEATURE_SEL_EndpointHalt       = 0x00,
    FEATURE_SEL_DeviceRemoteWakeup = 0x01,
    FEATURE_SEL_TestMode = 0x02,
};

#define GCC_MEMORY_BARRIER()                  __asm__ __volatile__("" ::: "memory");
#define GCC_IS_COMPILE_CONST(x)               __builtin_constant_p(x)

#define ATTR_WARN_UNUSED_RESULT      __attribute__ ((warn_unused_result))
#define ATTR_NON_NULL_PTR_ARG(...)   __attribute__ ((nonnull (__VA_ARGS__)))
#define ATTR_ALWAYS_INLINE           __attribute__ ((always_inline))
#define ATTR_CONST                   __attribute__ ((const))
#define ATTR_WEAK                    __attribute__ ((weak))
#define ATTR_NO_INIT                     __attribute__ ((section (".noinit")))
#define ATTR_ALIAS(Func)                 __attribute__ ((alias( #Func )))
#define ATTR_PACKED                      __attribute__ ((packed))

#if defined(USE_LUFA_CONFIG_HEADER)
#define USB_DEVICE_ONLY
#define USE_FLASH_DESCRIPTORS
static const uint8_t FIXED_CONTROL_ENDPOINT_SIZE = 8;
#define DEVICE_STATE_AS_GPIOR            0
static const uint8_t FIXED_NUM_CONFIGURATIONS = 1;
#endif

typedef uint8_t uint_reg_t;

static constexpr uint16_t le16_to_cpu(uint16_t x) { return x; }
static constexpr uint32_t le32_to_cpu(uint32_t x) { return x; }
static constexpr uint16_t cpu_to_le16(uint16_t x) { return x; }
static constexpr uint32_t cpu_to_le32(uint32_t x) { return x; }
static constexpr uint16_t LE16_TO_CPU(uint16_t x) { return x; }
static constexpr uint32_t LE32_TO_CPU(uint32_t x) { return x; }
static constexpr uint16_t CPU_TO_LE16(uint16_t x) { return x; }
static constexpr uint32_t CPU_TO_LE32(uint32_t x) { return x; }

static const uint8_t NO_DESCRIPTOR = 0;
#define USB_CONFIG_POWER_MA(mA)           ((mA) >> 1)
#define USB_STRING_LEN(UnicodeChars) (sizeof(USB_Descriptor_Header_t) + ((UnicodeChars) << 1))

#define VERSION_BCD(x)   CPU_TO_LE16((VERSION_TENS(x) << 12)  | (VERSION_ONES(x) << 8) | \
                              (VERSION_TENTHS(x) << 4) | (VERSION_HUNDREDTHS(x) << 0) )



enum USB_DescriptorTypes_t
{
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
};

enum USB_Descriptor_ClassSubclassProtocol_t
{
    USB_CSCP_NoDeviceClass          = 0x00,
    USB_CSCP_NoDeviceSubclass       = 0x00,
    USB_CSCP_NoDeviceProtocol       = 0x00,
    USB_CSCP_VendorSpecificClass    = 0xFF,
    USB_CSCP_VendorSpecificSubclass = 0xFF,
    USB_CSCP_VendorSpecificProtocol = 0xFF,
    USB_CSCP_IADDeviceClass         = 0xEF,
    USB_CSCP_IADDeviceSubclass      = 0x02,
    USB_CSCP_IADDeviceProtocol      = 0x01,
};

typedef struct
{
    uint8_t Size;
    uint8_t Type;
} ATTR_PACKED USB_Descriptor_Header_t;

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
} ATTR_PACKED USB_StdDescriptor_Header_t;

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

struct USB_StdDescriptor_Device_t
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
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
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint8_t bNumConfigurations;
    uint8_t bReserved;
} ATTR_PACKED USB_StdDescriptor_DeviceQualifier_t;

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
uint8_t bLength;
uint8_t bDescriptorType;
uint16_t wTotalLength;
uint8_t bNumInterfaces;
uint8_t bConfigurationValue;
uint8_t iConfiguration;
uint8_t bmAttributes;
uint8_t bMaxPower;
} ATTR_PACKED USB_StdDescriptor_Configuration_Header_t;

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

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} ATTR_PACKED USB_StdDescriptor_Interface_t;

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

struct USB_StdDescriptor_Interface_Association_t
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bFirstInterface;
    uint8_t bInterfaceCount;
    uint8_t bFunctionClass;
    uint8_t bFunctionSubClass;
    uint8_t bFunctionProtocol;
    uint8_t iFunction;
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
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} ATTR_PACKED USB_StdDescriptor_Endpoint_t;

typedef struct
{
    USB_Descriptor_Header_t Header;
    wchar_t  UnicodeString[];
} ATTR_PACKED USB_Descriptor_String_t;

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bString[];
} ATTR_PACKED USB_StdDescriptor_String_t;

#define VERSION_TENS(x)                   (int)((int)(x) / 10)
#define VERSION_ONES(x)                   (int)((int)(x) % 10)
#define VERSION_TENTHS(x)                 (int)((x - (int)x) * 10)
#define VERSION_HUNDREDTHS(x)             (int)((x * 100) - ((int)(x * 10) * 10))



enum USB_Device_States_t
{
    DEVICE_STATE_Unattached  = 0,
    DEVICE_STATE_Powered = 1,
    DEVICE_STATE_Default = 2,
    DEVICE_STATE_Addressed = 3,
    DEVICE_STATE_Configured = 4,
    DEVICE_STATE_Suspended = 5,
};

enum USB_Modes_t
{
    USB_MODE_None   = 0,
    USB_MODE_Device = 1,
    USB_MODE_Host   = 2,
    USB_MODE_UID    = 3,
};

static const uint8_t USB_DEVICE_OPT_FULLSPEED = 0<<0;
static const uint8_t USE_INTERNAL_SERIAL = 0xDC;
static const uint8_t INTERNAL_SERIAL_LENGTH_BITS = 80;
static const uint8_t INTERNAL_SERIAL_START_ADDRESS = 0x0e;

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue, const uint8_t wIndex,
       const void** const DescriptorAddress) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);





enum Endpoint_Stream_RW_ErrorCodes_t
{
    ENDPOINT_RWSTREAM_NoError            = 0,
    ENDPOINT_RWSTREAM_EndpointStalled    = 1,
    ENDPOINT_RWSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWSTREAM_BusSuspended       = 3,
    ENDPOINT_RWSTREAM_Timeout            = 4,
    ENDPOINT_RWSTREAM_IncompleteTransfer = 5,
};

enum Endpoint_ControlStream_RW_ErrorCodes_t
{
    ENDPOINT_RWCSTREAM_NoError            = 0,
    ENDPOINT_RWCSTREAM_HostAborted        = 1,
    ENDPOINT_RWCSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWCSTREAM_BusSuspended       = 3,
};

extern volatile bool USB_IsInitialized;
extern USB_Request_Header_t USB_ControlRequest;

#define _GET_DEVICE_GPIOR_NAME2(y) GPIOR ## y
#define _GET_DEVICE_GPIOR_NAME(x) _GET_DEVICE_GPIOR_NAME2(x)
#define USB_DeviceState _GET_DEVICE_GPIOR_NAME(DEVICE_STATE_AS_GPIOR)

typedef struct
{
    uint8_t  Address;
    uint16_t Size;
    uint8_t  Type;
    uint8_t  Banks;
} USB_Endpoint_Table_t;

static const uint8_t ENDPOINT_EPNUM_MASK = 0x0f;
static const uint8_t ENDPOINT_CONTROLEP = 0;
static const uint8_t ENDPOINT_TOTAL_ENDPOINTS = 7;

enum Endpoint_WaitUntilReady_ErrorCodes_t
{
    ENDPOINT_READYWAIT_NoError = 0,
    ENDPOINT_READYWAIT_EndpointStalled = 1,
    ENDPOINT_READYWAIT_DeviceDisconnected = 2,
    ENDPOINT_READYWAIT_BusSuspended = 3,
    ENDPOINT_READYWAIT_Timeout = 4,
};

struct CDC_LineEncoding_t
{
    uint32_t BaudRateBPS;
    uint8_t CharFormat;
    uint8_t ParityType;
    uint8_t DataBits;
} __attribute__ ((packed));

typedef struct
{
    struct
    {
        uint8_t ControlInterfaceNumber;
        USB_Endpoint_Table_t DataINEndpoint;
        USB_Endpoint_Table_t DataOUTEndpoint;
        USB_Endpoint_Table_t NotificationEndpoint;
    } Config;

    struct
    {
        struct
        {
            uint16_t HostToDevice;
            uint16_t DeviceToHost;
        } ControlLineStates;

        CDC_LineEncoding_t LineEncoding;

    } State;

} USB_ClassInfo_CDC_Device_t;

class USB : public Terminal
{
private:
    void createStream(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo, FILE* const Stream);
    void inline createStream() { createStream(&cdcDevice, &cdcStream); }
    FILE cdcStream;
    inline uint16_t Endpoint_Read_16_LE();
    inline uint16_t Endpoint_Read_16_BE();
    inline bool USB_Device_IsAddressSet() { return UDADDR & 1<<ADDEN; }
    void Endpoint_ClearStatusStage();
    inline void Endpoint_EnableEndpoint() { UECONX |= 1<<EPEN; }
    inline void Endpoint_DisableEndpoint() { UECONX &= ~(1<<EPEN); }
    inline bool Endpoint_IsEnabled() { return UECONX & 1<<EPEN ? true : false; }
    inline uint8_t Endpoint_GetBusyBanks() { return UESTA0X & 3<<NBUSYBK0; }
    inline void Endpoint_AbortPendingIN();
    uint8_t Endpoint_Write_Control_Stream_BE(const void* const buffer, uint16_t length);
    uint8_t Endpoint_Read_Control_Stream_LE(void* const Buffer, uint16_t Length);
    uint8_t Endpoint_Read_Control_Stream_BE(void* const Buffer, uint16_t Length);
    uint8_t Endpoint_Write_Control_EStream_LE(const void* const Buffer, uint16_t len);
    uint8_t Endpoint_Write_Control_EStream_BE(const void* const Buffer, uint16_t len);
    uint8_t Endpoint_Read_Control_EStream_LE(void* const Buffer, uint16_t len);
    uint8_t Endpoint_Read_Control_EStream_BE(void* const Buffer, uint16_t len);
    static inline void Endpoint_ClearIN() { UEINTX &= ~(1<<TXINI | 1<<FIFOCON); }
    static inline void Endpoint_ClearOUT() { UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON); }
    inline void Endpoint_StallTransaction() { UECONX |= 1<<STALLRQ; }
    inline void Endpoint_ClearStall(){ UECONX |= 1<<STALLRQC; }
    static inline bool Endpoint_IsStalled() { return UECONX & 1<<STALLRQ ? true : false; }
    inline void Endpoint_ResetDataToggle() { UECONX |= 1<<RSTDT; }
    static inline bool Endpoint_IsINReady() { return UEINTX & 1<<TXINI ? true : false; }
    static inline bool Endpoint_IsOUTReceived() { return UEINTX & 1<<RXOUTI ? true : false; }
    static inline bool Endpoint_IsSETUPReceived() { return UEINTX & 1<<RXSTPI ? true : false; }
    static inline bool Endpoint_IsReadWriteAllowed() { return UEINTX & 1<<RWAL ? true : false; }
    inline bool Endpoint_IsConfigured() { return UESTA0X & 1<<CFGOK ? true : false; }
    inline uint8_t Endpoint_GetEndpointInterrupts() { return UEINT; }
    inline bool Endpoint_HasEndpointInterrupted(const uint8_t Address);
    static inline void Endpoint_ClearSETUP() { UEINTX &= ~(1<<RXSTPI); }
    inline void Endpoint_SetEndpointDirection(const uint8_t DirectionMask);
    static inline uint8_t Endpoint_Read_8() { return UEDATX; }
    static inline uint16_t USB_Device_GetFrameNumber() { return UDFNUM; }
    inline void USB_Device_EnableSOFEvents() { USB_INT_Enable(USB_INT_SOFI); }
    inline void USB_Device_DisableSOFEvents() { USB_INT_Disable(USB_INT_SOFI); }
    inline void USB_Device_SetLowSpeed() { UDCON |= 1<<LSM; }
    inline void USB_Device_SetFullSpeed() { UDCON &= ~(1<<LSM); }
    static inline void Endpoint_Write_8(const uint8_t Data) { UEDATX = Data; }
    inline void Endpoint_Discard_8();
    inline void Endpoint_Write_16_LE(const uint16_t Data);
    inline void Endpoint_Write_16_BE(const uint16_t Data);
    inline void Endpoint_Discard_16();
    inline uint32_t Endpoint_Read_32_LE();
    inline uint32_t Endpoint_Read_32_BE();
    inline void Endpoint_Write_32_LE(const uint32_t Data);
    uint8_t Endpoint_Null_Stream(uint16_t Length, uint16_t* const BytesProcessed);
    inline void Endpoint_Write_32_BE(const uint32_t Data);
    inline void Endpoint_Discard_32();
    void CDC_Device_USBTask(USB_ClassInfo_CDC_Device_t * const CDCInterfaceInfo);
    inline void disableAllInterrupts() { USBCON &= ~(1<<VBUSTE); UDIEN = 0; }
    inline void clearAllInterrupts() { USBINT = 0; }
    static USB_ClassInfo_CDC_Device_t cdcDevice;
    uint8_t CDC_Device_Flush(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
    void inline cdcTask() { CDC_Device_USBTask(&cdcDevice); }
    inline void USB_INT_Clear(const uint8_t Interrupt);
    inline bool USB_INT_IsEnabled(const uint8_t Interrupt);
    inline void USB_OTGPAD_Off() const { USBCON &= ~(1<<OTGPADE); }
    inline void USB_CLK_Freeze() { USBCON |= 1<<FRZCLK; }
    inline void USB_CLK_Unfreeze() { USBCON &= ~(1<<FRZCLK);}
    inline void USB_Controller_Enable() { USBCON |= 1<<USBE; }
    inline void USB_Controller_Disable() { USBCON &= ~(1<<USBE); }
    inline void USB_Controller_Reset() { USBCON &= ~(1<<USBE); USBCON |= 1<<USBE; }
    inline bool USB_VBUS_GetStatus() { return USBSTA & 1<<VBUS ? true : false; }
    inline void USB_Detach() { UDCON |=  1<<DETACH; }
    inline void USB_Attach() { UDCON  &= ~(1<<DETACH); }
    inline void USB_PLL_On() { PLLCSR = USB_PLL_PSC; PLLCSR = USB_PLL_PSC | 1<<PLLE; }
    inline void USB_PLL_Off() { PLLCSR = 0; }
    inline bool USB_PLL_IsReady() { return PLLCSR & 1<<PLOCK ? true : false; }
    inline void USB_REG_On() { UHWCON |= 1<<UVREGE; }
    inline void USB_REG_Off(){ UHWCON &= ~(1<<UVREGE); }
    inline void USB_OTGPAD_On() { USBCON |= 1<<OTGPADE; }
    void USB_INT_Enable(uint8_t Interrupt);
    void USB_INT_Disable(uint8_t Interrupt);
    bool USB_INT_HasOccurred(uint8_t Interrupt);
    static inline uint16_t Endpoint_BytesInEndpoint() { return ((uint16_t)UEBCHX<<8) | UEBCLX; }
    inline void Device_GetSerialString(uint16_t* const UnicodeString);
    void Endpoint_ResetEndpoint(const uint8_t addr);
    static uint8_t Endpoint_GetEndpointDirection();
    bool Endpoint_ConfigureEndpoint_Prv(uint8_t Number, uint8_t UECFG0XData, uint8_t UECFG1XData);
    void USB_Device_SetDeviceAddress(const uint8_t Address);
    static void Endpoint_SelectEndpoint(uint8_t addr) { UENUM = addr & ENDPOINT_EPNUM_MASK; }
    uint8_t Endpoint_GetCurrentEndpoint();
    static void SetGlobalInterruptMask(const uint_reg_t GlobalIntState);
    static void GlobalInterruptEnable();
    static void GlobalInterruptDisable();
    static inline uint_reg_t GetGlobalInterruptMask() { GCC_MEMORY_BARRIER(); return SREG; }
    void EVENT_USB_Device_ConfigurationChanged();
    bool CDC_Device_ConfigureEndpoints(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
    static int CDC_Device_putchar(char c, FILE* Stream);
    static int CDC_Device_getchar(FILE* Stream);
    void Device_SetConfiguration();
    void Device_GetConfiguration();
    void Device_GetStatus();
    uint8_t Endpoint_Write_Control_PStream_LE(const void* const Buffer, uint16_t Length);
    void Device_GetInternalSerialDescriptor();
    static uint8_t Endpoint_WaitUntilReady();
    static int16_t CDC_Device_ReceiveByte(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
    void EVENT_USB_Device_ControlRequest();
    void CDC_Device_ProcessControlRequest(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
    void Device_ClearSetFeature();
    static uint8_t Endpoint_Write_Control_Stream_LE(const void* const buffer, uint16_t length);
public:
    void initDevice();
    void myPutc(char c);
    uint8_t readByte();
    void resetInterface();
    USB();
    void inline flush() { CDC_Device_Flush(&cdcDevice); }
    void init();
    void Device_ProcessControlRequest();
    void task() { USB_DeviceTask(); }
    void USB_DeviceTask();
    void Device_GetDescriptor();
    void Device_SetAddress();
    void gen();
    void com();
    static USB *instance;
private:
    static uint8_t CDC_Device_SendByte(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo,
                            const uint8_t Data);

    static constexpr uint8_t USB_Options = USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED |
            USB_OPT_AUTO_PLL;

    inline void USB_Device_EnableDeviceAddress(const uint8_t Address)
    { (void)Address; UDADDR |= 1<<ADDEN; }

    uint8_t Endpoint_Write_Stream_LE(const void* const Buffer, uint16_t Length,
                       uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

    uint8_t Endpoint_Write_Stream_BE(const void* const Buffer, uint16_t Length,
                       uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

    uint8_t Endpoint_Read_Stream_LE(void* const Buffer,  uint16_t Length,
                        uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

    uint8_t Endpoint_Read_Stream_BE(void* const Buffer, uint16_t Length,
                                  uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

    bool Endpoint_ConfigureEndpoint(const uint8_t Address, const uint8_t type,
        const uint16_t size, const uint8_t banks);

    static inline uint8_t Endpoint_BytesToEPSizeMask(const uint16_t Bytes)
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

    bool Endpoint_ConfigureEndpointTable(const USB_Endpoint_Table_t* const Table,
                                         const uint8_t Entries);
};

#endif

