#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <math.h>
#include <util/delay.h>






#define ATTR_PACKED                      __attribute__ ((packed))






#define ENDPOINT_DIR_MASK                  0x80
#define ENDPOINT_DIR_OUT                   0x00
#define ENDPOINT_DIR_IN                    0x80
#define EP_TYPE_CONTROL                    0x00
#define EP_TYPE_ISOCHRONOUS                0x01
#define EP_TYPE_BULK                       0x02
#define EP_TYPE_INTERRUPT                  0x03

enum USB_Modes_t
{
    USB_MODE_None   = 0,
    USB_MODE_Device = 1,
    USB_MODE_Host   = 2,
    USB_MODE_UID    = 3,
};

enum USB_Device_States_t
{
    DEVICE_STATE_Unattached = 0,
    DEVICE_STATE_Powered = 1,
    DEVICE_STATE_Default = 2,
    DEVICE_STATE_Addressed  = 3,
    DEVICE_STATE_Configured = 4,
    DEVICE_STATE_Suspended = 5,
};






#define NO_DESCRIPTOR    0
#define USB_CONFIG_POWER_MA(mA)           ((mA) >> 1)

#define USB_STRING_LEN(UnicodeChars) (sizeof(USB_Descriptor_Header_t) + ((UnicodeChars) << 1))

#define USB_CONFIG_ATTR_RESERVED          0x80
#define USB_CONFIG_ATTR_SELFPOWERED       0x40
#define USB_CONFIG_ATTR_REMOTEWAKEUP      0x20
#define ENDPOINT_ATTR_NO_SYNC             (0 << 2)
#define ENDPOINT_ATTR_ASYNC               (1 << 2)
#define ENDPOINT_ATTR_ADAPTIVE            (2 << 2)
#define ENDPOINT_ATTR_SYNC                (3 << 2)
#define ENDPOINT_USAGE_DATA               (0 << 4)
#define ENDPOINT_USAGE_FEEDBACK           (1 << 4)

enum USB_DescriptorTypes_t
{
    DTYPE_Device = 0x01,
    DTYPE_Configuration = 0x02,
    DTYPE_String = 0x03,
    DTYPE_Interface = 0x04,
    DTYPE_Endpoint = 0x05,
    DTYPE_DeviceQualifier = 0x06,
    DTYPE_Other = 0x07,
    DTYPE_InterfacePower = 0x08,
    DTYPE_InterfaceAssociation = 0x0B,
    DTYPE_CSInterface = 0x24,
    DTYPE_CSEndpoint = 0x25,
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

struct USB_Descriptor_Header_t
{
    uint8_t Size;
    uint8_t Type;
}
__attribute__ ((packed)) ;
			
struct USB_StdDescriptor_Header_t
{
    uint8_t bLength; /**< Size of the descriptor, in bytes. */
    uint8_t bDescriptorType;
}
__attribute__ ((packed));

typedef struct
{
USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
uint16_t USBSpecification;
uint8_t  Class; /**< USB device class. */
uint8_t  SubClass; /**< USB device subclass. */
uint8_t  Protocol; /**< USB device protocol. */
uint8_t  Endpoint0Size; /**< Size of the control (address 0) endpoint's bank in bytes. */
uint16_t VendorID; /**< Vendor ID for the USB product. */
uint16_t ProductID; /**< Unique product ID for the USB product. */
uint16_t ReleaseNumber;
uint8_t  ManufacturerStrIndex;
uint8_t  ProductStrIndex;
uint8_t  SerialNumStrIndex;
uint8_t  NumberOfConfigurations;
} ATTR_PACKED USB_Descriptor_Device_t;

typedef struct
{
uint8_t  bLength; /**< Size of the descriptor, in bytes. */
uint8_t  bDescriptorType;
uint16_t bcdUSB;
uint8_t  bDeviceClass; /**< USB device class. */
uint8_t  bDeviceSubClass; /**< USB device subclass. */
uint8_t  bDeviceProtocol; /**< USB device protocol. */
uint8_t  bMaxPacketSize0; /**< Size of the control (address 0) endpoint's bank in bytes. */
uint16_t idVendor; /**< Vendor ID for the USB product. */
uint16_t idProduct; /**< Unique product ID for the USB product. */
uint16_t bcdDevice;
uint8_t  iManufacturer;
uint8_t  iProduct;
uint8_t iSerialNumber;
uint8_t  bNumConfigurations;
} ATTR_PACKED USB_StdDescriptor_Device_t;

typedef struct
{
USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
uint16_t USBSpecification;
uint8_t  Class; /**< USB device class. */
uint8_t  SubClass; /**< USB device subclass. */
uint8_t  Protocol; /**< USB device protocol. */
uint8_t  Endpoint0Size; /**< Size of the control (address 0) endpoint's bank in bytes. */
uint8_t  NumberOfConfigurations;
uint8_t  Reserved; /**< Reserved for future use, must be 0. */
} ATTR_PACKED USB_Descriptor_DeviceQualifier_t;

struct USB_StdDescriptor_DeviceQualifier_t
{
uint8_t  bLength; /**< Size of the descriptor, in bytes. */
uint8_t  bDescriptorType;
uint16_t bcdUSB;
uint8_t  bDeviceClass; /**< USB device class. */
uint8_t  bDeviceSubClass; /**< USB device subclass. */
uint8_t  bDeviceProtocol; /**< USB device protocol. */
uint8_t  bMaxPacketSize0; /**< Size of the control (address 0) endpoint's bank in bytes. */
uint8_t  bNumConfigurations;
uint8_t  bReserved; /**< Reserved for future use, must be 0. */
} ATTR_PACKED;

typedef struct
{
    USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
    uint16_t TotalConfigurationSize;
    uint8_t  TotalInterfaces;
    uint8_t  ConfigurationNumber;
    uint8_t  ConfigurationStrIndex; /**< Index  */
    uint8_t  ConfigAttributes;
    uint8_t  MaxPowerConsumption;
} ATTR_PACKED USB_Descriptor_Configuration_Header_t;

struct USB_StdDescriptor_Configuration_Header_t
{
    uint8_t  bLength; /**< Size of the descriptor, in bytes. */
    uint8_t  bDescriptorType;
    uint16_t wTotalLength; /**< Size of the configuration descriptor heade*/
    uint8_t  bNumInterfaces; /**< Total number of interfaces in the configuration. */
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes; /**< Configuration sed of a mask of \c USB_CONFIG_ATTR_* masks.*/
    uint8_t  bMaxPower; /**< Maximum power consumption of the device while in */
}
ATTR_PACKED USB_StdDescriptor_Configuration_Header_t;

			
typedef struct
{
USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
uint8_t InterfaceNumber; /**< Index of the interface in the current configuration. */
uint8_t AlternateSetting;
uint8_t TotalEndpoints; /**< Total number of endpoints in the interface. */
uint8_t Class; /**< Interface class ID. */
uint8_t SubClass; /**< Interface subclass ID. */
uint8_t Protocol; /**< Interface protocol ID. */
uint8_t InterfaceStrIndex; /**< Index of the string descriptor describing the interface. */
} ATTR_PACKED USB_Descriptor_Interface_t;

typedef struct
{
    uint8_t bLength; /**< Size of the descriptor, in bytes. */
    uint8_t bDescriptorType; /**< iptor, either a value in \ref USB_DescriptorTypes_t or a va*/
    uint8_t bInterfaceNumber; /**< Index of the interface in the current configuration. */
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints; /**< Total number of endpoints in the interface. */
    uint8_t bInterfaceClass; /**< Interface class ID. */
    uint8_t bInterfaceSubClass; /**< Interface subclass ID. */
    uint8_t bInterfaceProtocol; /**< Interface protocol ID. */
    uint8_t iInterface;
} ATTR_PACKED USB_StdDescriptor_Interface_t;

typedef struct
{
    USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
    uint8_t FirstInterfaceIndex; /**< Index of the first associated interface. */
    uint8_t TotalInterfaces; /**< Total number of associated interfaces. */
    uint8_t Class; /**< Interface class ID. */
    uint8_t SubClass; /**< Interface subclass ID. */
    uint8_t Protocol; /**< Interface protocol ID. */
    uint8_t IADStrIndex;
} ATTR_PACKED USB_Descriptor_Interface_Association_t;

typedef struct
{
uint8_t bLength; /**< Size of the descriptor, in bytes. */
uint8_t bDescriptorType;
uint8_t bFirstInterface; /**< Index of the first associated interface. */
uint8_t bInterfaceCount; /**< Total number of associated interfaces. */
uint8_t bFunctionClass; /**< Interface class ID. */
uint8_t bFunctionSubClass; /**< Interface subclass ID. */
uint8_t bFunctionProtocol; /**< Interface protocol ID. */
uint8_t iFunction;
} ATTR_PACKED USB_StdDescriptor_Interface_Association_t;

typedef struct
{
    USB_Descriptor_Header_t Header;
    uint8_t EndpointAddress; /**< Logical nt within the device for the current*/
    uint8_t  Attributes;
    uint16_t EndpointSize;
    uint8_t  PollingIntervalMS;
} ATTR_PACKED USB_Descriptor_Endpoint_t;

struct USB_StdDescriptor_Endpoint_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType; 
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} ATTR_PACKED;
			
template <size_t S> struct USB_Descriptor_String_t
{
    USB_Descriptor_Header_t Header;
    wchar_t UnicodeString[S];
} ATTR_PACKED;

typedef struct
{
    uint8_t bLength; /**< Size of the descriptor, in bytes. */
    uint8_t bDescriptorType;
    uint16_t bString[];
} ATTR_PACKED USB_StdDescriptor_String_t;

enum USB_Interrupts_t
{
    USB_INT_VBUSTI  = 0,
    USB_INT_WAKEUPI = 2,
    USB_INT_SUSPI   = 3,
    USB_INT_EORSTI  = 4,
    USB_INT_SOFI    = 5,
    USB_INT_RXSTPI  = 6,
};

typedef struct
{
    uint8_t  Address; /**< Addresto configure, or zero if the table entry is to be unused. */
    uint16_t Size; /**< Size of the endpoint bank, in bytes. */
    uint8_t  Type; /**< Type of the endpoint, a \c EP_TYPE_* mask. */
    uint8_t  Banks; /**< Number of hardware banks to use for the endpoint. */
} USB_Endpoint_Table_t;

#define ENDPOINT_EPNUM_MASK                     0x0F
#define ENDPOINT_CONTROLEP                      0

static inline uint8_t Endpoint_BytesToEPSizeMask(const uint16_t Bytes)
{
    uint8_t  MaskVal    = 0;
    uint16_t CheckBytes = 8;

    while (CheckBytes < Bytes)
    {
        MaskVal++;
        CheckBytes <<= 1;
    }

    return MaskVal << EPSIZE0;
}

static constexpr uint8_t ENDPOINT_TOTAL_ENDPOINTS = 7;

enum Endpoint_WaitUntilReady_ErrorCodes_t
{
    ENDPOINT_READYWAIT_NoError = 0, /**< Endpoint is ready for next packet, no error. */
    ENDPOINT_READYWAIT_EndpointStalled = 1, /**< The endpoint was stalled during the stream*/
    ENDPOINT_READYWAIT_DeviceDisconnected  = 2,
    ENDPOINT_READYWAIT_BusSuspended = 3,
    ENDPOINT_READYWAIT_Timeout = 4,
};



static inline uint16_t Endpoint_BytesInEndpoint(void)
{
    return (((uint16_t)UEBCHX << 8) | UEBCLX);
}

static inline uint8_t Endpoint_GetEndpointDirection(void)
{
    return (UECFG0X & (1 << EPDIR)) ? ENDPOINT_DIR_IN : ENDPOINT_DIR_OUT;
}

static inline uint8_t Endpoint_GetCurrentEndpoint(void)
{
    return ((UENUM & ENDPOINT_EPNUM_MASK) | Endpoint_GetEndpointDirection());
}

static inline void Endpoint_SelectEndpoint(const uint8_t Address)
{
    UENUM = (Address & ENDPOINT_EPNUM_MASK);
}

static inline void Endpoint_ResetEndpoint(const uint8_t Address)
{
    UERST = (1 << (Address & ENDPOINT_EPNUM_MASK));
    UERST = 0;
}

static inline bool Endpoint_HasEndpointInterrupted(const uint8_t Address)
{
    return ((UEINT & (1<<(Address & ENDPOINT_EPNUM_MASK))) ? true : false);
}

static inline bool Endpoint_IsINReady(void)
{
    return ((UEINTX & (1 << TXINI)) ? true : false);
}

static inline void Endpoint_SetEndpointDirection(const uint8_t DirectionMask)
{   
    UECFG0X = ((UECFG0X & ~(1 << EPDIR)) | (DirectionMask ? (1 << EPDIR) : 0));
}   

static inline uint16_t Endpoint_Read_16_LE(void)
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


bool Endpoint_ConfigureEndpointTable(const USB_Endpoint_Table_t* const Table,
                                                 const uint8_t Entries);


#define USB_DEVICE_OPT_LOWSPEED            (1 << 0)
#define USB_DEVICE_OPT_FULLSPEED               (0 << 0)


#define USE_INTERNAL_SERIAL            NO_DESCRIPTOR
#define INTERNAL_SERIAL_LENGTH_BITS    0
#define INTERNAL_SERIAL_START_ADDRESS  0

void USB_Device_SendRemoteWakeup(void);

static inline void USB_Device_SetDeviceAddress(const uint8_t Address)
{
    UDADDR = (UDADDR & (1 << ADDEN)) | (Address & 0x7F);
}

static inline void USB_Device_EnableDeviceAddress(const uint8_t Address)
{
    (void)Address;
    UDADDR |= (1 << ADDEN);
}

static inline void USB_Device_GetSerialString(uint16_t* const UnicodeString)
{
    __asm__ __volatile__("" ::: "memory");
    uint8_t CurrentGlobalInt = SREG;
    cli();

    uint8_t SigReadAddress = INTERNAL_SERIAL_START_ADDRESS;

    for (uint8_t SerialCharNum = 0;
        SerialCharNum < (INTERNAL_SERIAL_LENGTH_BITS / 4);
        SerialCharNum++)
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

    __asm__ __volatile__("" ::: "memory");
    SREG = CurrentGlobalInt;
    __asm__ __volatile__("" ::: "memory");
}

#define CONTROL_REQTYPE_DIRECTION  0x80
#define CONTROL_REQTYPE_TYPE       0x60
#define CONTROL_REQTYPE_RECIPIENT  0x1F
#define REQDIR_HOSTTODEVICE        (0 << 7)
#define REQDIR_DEVICETOHOST        (1 << 7)
#define REQTYPE_STANDARD           (0 << 5)
#define REQTYPE_CLASS              (1 << 5)
#define REQTYPE_VENDOR             (2 << 5)
#define REQREC_DEVICE              (0 << 0)
#define REQREC_INTERFACE           (1 << 0)
#define REQREC_ENDPOINT            (2 << 0)
#define REQREC_OTHER               (3 << 0)

			
struct USB_Request_Header_t
{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} ATTR_PACKED;

enum USB_Control_Request_t
{
REQ_GetStatus           = 0,
REQ_ClearFeature        = 1,
REQ_SetFeature          = 3,
REQ_SetAddress          = 5,
REQ_GetDescriptor       = 6,
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
FEATURE_SEL_TestMode           = 0x02,
};

#define FEATURE_SELFPOWERED_ENABLED     (1 << 0)
#define FEATURE_REMOTE_WAKEUP_ENABLED   (1 << 1)


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
    ENDPOINT_RWCSTREAM_NoError = 0, /**< Command completed successfully, no error. */
    ENDPOINT_RWCSTREAM_HostAborted = 1, /**< The aborted the transfer prematurely. */
    ENDPOINT_RWCSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWCSTREAM_BusSuspended = 3,
};



#if !defined(USB_STREAM_TIMEOUT_MS) || defined(__DOXYGEN__)
#define USB_STREAM_TIMEOUT_MS       100
#endif


enum HID_Parse_ErrorCodes_t
{
    HID_PARSE_Successful = 0, /**< Successful parse of the HID report descriptor, no error. */
    HID_PARSE_HIDStackOverflow = 1,
    HID_PARSE_HIDStackUnderflow = 2,
    HID_PARSE_InsufficientReportItems = 3,
    HID_PARSE_UnexpectedEndCollection = 4,
    HID_PARSE_InsufficientCollectionPaths = 5, /**< MOLLECTIONS collections in the report. */
    HID_PARSE_UsageListOverflow           = 6, /**< TH usages listed in a row. */
    HID_PARSE_InsufficientReportIDItems   = 7, /**< eport IDs in the device. */
    HID_PARSE_NoUnfilteredReportItems     = 8, /**< iltering callback routine. */
};

typedef struct
{
    uint32_t Minimum;
    uint32_t Maximum;
} HID_MinMax_t;

typedef struct
{
    uint32_t Type;
    uint8_t  Exponent;
} HID_Unit_t;

typedef struct
{
    uint16_t Page;  /**< Usage page of the report item. */
    uint16_t Usage; /**< Usage of the report item. */
} HID_Usage_t;

typedef struct HID_CollectionPath
{
    uint8_t                    Type;   /**< Collection type (e.g. "Generic Desktop"). */
    HID_Usage_t                Usage;  /**< Collection usage. */
    struct HID_CollectionPath* Parent; /**< Reference ection, or \c NULL if root collection. */
} HID_CollectionPath_t;

typedef struct
{
    uint8_t      BitSize;
    HID_Usage_t  Usage;
    HID_Unit_t   Unit;
    HID_MinMax_t Logical;  
    HID_MinMax_t Physical; 
} HID_ReportItem_Attributes_t;

typedef struct
{
    uint16_t BitOffset;   /**< Bit offset in the IN, OUT or FEATURE report of the item. */
    uint8_t                     ItemType;       /**< Report item type, a valItemTypes_t. */
    uint16_t                    ItemFlags;      /**< Item data fOF_* constants. */
    uint8_t                     ReportID;       /**< Report ID tnly one report */
    HID_CollectionPath_t*       CollectionPath; /**< Collection path of the item. */
    HID_ReportItem_Attributes_t Attributes;     /**< Report item attributes. */
    uint32_t                    Value;
    uint32_t                    PreviousValue;  /**< Previous value of the report item. */
            } HID_ReportItem_t;

typedef struct
{
    uint8_t  ReportID; /**< Report ID of the report within the HID interface. */
    uint16_t ReportSizeBits[3];
} HID_ReportSizeInfo_t;

typedef struct
{   
    uint8_t TotalReportItems; /**< Total rt items stored in the \c ReportItems array. */
    HID_ReportItem_t     ReportItems[20];
    HID_CollectionPath_t CollectionPaths[10];
    uint8_t              TotalDeviceReports;
    HID_ReportSizeInfo_t ReportIDSizes[10];
    uint16_t             LargestReportSizeBits;
    bool                 UsingReportIDs;
} HID_ReportInfo_t;


bool CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t* const CurrentItem);

typedef struct
{
    HID_ReportItem_Attributes_t Attributes;
    uint8_t                     ReportCount;
    uint8_t                     ReportID;
} HID_StateTable_t;

static const uint8_t
    HID_KEYBOARD_MODIFIER_LEFTCTRL = 1<<0,
    HID_KEYBOARD_MODIFIER_LEFTSHIFT = 1<<1,
    HID_KEYBOARD_MODIFIER_LEFTALT    = 1<<2,
    HID_KEYBOARD_MODIFIER_LEFTGUI    = 1<<3,
    HID_KEYBOARD_MODIFIER_RIGHTCTRL  = 1<<4,
    HID_KEYBOARD_MODIFIER_RIGHTSHIFT = 1<<5,
    HID_KEYBOARD_MODIFIER_RIGHTALT   = 1<<6,
    HID_KEYBOARD_MODIFIER_RIGHTGUI   = 1<<7,
    HID_KEYBOARD_LED_NUMLOCK         = 1<<0,
    HID_KEYBOARD_LED_CAPSLOCK        = 1<<1,
    HID_KEYBOARD_LED_SCROLLLOCK      = 1<<2,
    HID_KEYBOARD_LED_COMPOSE         = 1<<3,
    HID_KEYBOARD_LED_KANA            = 1<<4,
    HID_KEYBOARD_SC_ERROR_ROLLOVER                  =  0x01,
    HID_KEYBOARD_SC_POST_FAIL                       =  0x02,
    HID_KEYBOARD_SC_ERROR_UNDEFINED                 =  0x03,
    HID_KEYBOARD_SC_A =  0x04,
    HID_KEYBOARD_SC_B =  0x05,
    HID_KEYBOARD_SC_C =  0x06,
    HID_KEYBOARD_SC_D =  0x07,
    HID_KEYBOARD_SC_E =  0x08,
    HID_KEYBOARD_SC_F =  0x09,
    HID_KEYBOARD_SC_G =  0x0A,
    HID_KEYBOARD_SC_H =  0x0B,
    HID_KEYBOARD_SC_I =  0x0C,
    HID_KEYBOARD_SC_J =  0x0D,
    HID_KEYBOARD_SC_K =  0x0E,
    HID_KEYBOARD_SC_L =  0x0F,
    HID_KEYBOARD_SC_M =  0x10,
    HID_KEYBOARD_SC_N =  0x11,
    HID_KEYBOARD_SC_O =  0x12,
    HID_KEYBOARD_SC_P =  0x13,
    HID_KEYBOARD_SC_Q =  0x14,
    HID_KEYBOARD_SC_R =  0x15,
    HID_KEYBOARD_SC_S =  0x16,
    HID_KEYBOARD_SC_T =  0x17,
    HID_KEYBOARD_SC_U =  0x18,
    HID_KEYBOARD_SC_V =  0x19,
    HID_KEYBOARD_SC_W =  0x1A,
    HID_KEYBOARD_SC_X =  0x1B,
    HID_KEYBOARD_SC_Y =  0x1C,
    HID_KEYBOARD_SC_Z =  0x1D,
    HID_KEYBOARD_SC_1_AND_EXCLAMATION               =  0x1E,
    HID_KEYBOARD_SC_2_AND_AT                        =  0x1F,
    HID_KEYBOARD_SC_3_AND_HASHMARK                  =  0x20,
    HID_KEYBOARD_SC_4_AND_DOLLAR                    =  0x21,
    HID_KEYBOARD_SC_5_AND_PERCENTAGE                =  0x22,
    HID_KEYBOARD_SC_6_AND_CARET                     =  0x23,
    HID_KEYBOARD_SC_7_AND_AMPERSAND                 =  0x24,
    HID_KEYBOARD_SC_8_AND_ASTERISK                  =  0x25,
    HID_KEYBOARD_SC_9_AND_OPENING_PARENTHESIS       =  0x26,
    HID_KEYBOARD_SC_0_AND_CLOSING_PARENTHESIS       =  0x27,
    HID_KEYBOARD_SC_ENTER                           =  0x28,
    HID_KEYBOARD_SC_ESCAPE                          =  0x29,
    HID_KEYBOARD_SC_BACKSPACE                       =  0x2A,
    HID_KEYBOARD_SC_TAB                             =  0x2B,
    HID_KEYBOARD_SC_SPACE                           =  0x2C,
    HID_KEYBOARD_SC_MINUS_AND_UNDERSCORE            =  0x2D,
    HID_KEYBOARD_SC_EQUAL_AND_PLUS                  =  0x2E,
    HID_KEYBOARD_SC_OPENING_BRACKET_AND_OPENING_BRACE = 0x2F,
    HID_KEYBOARD_SC_CLOSING_BRACKET_AND_CLOSING_BRACE = 0x30,
    HID_KEYBOARD_SC_BACKSLASH_AND_PIPE              =  0x31,
    HID_KEYBOARD_SC_NON_US_HASHMARK_AND_TILDE       =  0x32,
    HID_KEYBOARD_SC_SEMICOLON_AND_COLON             =  0x33,
    HID_KEYBOARD_SC_APOSTROPHE_AND_QUOTE            =  0x34,
    HID_KEYBOARD_SC_GRAVE_ACCENT_AND_TILDE          =  0x35,
    HID_KEYBOARD_SC_COMMA_AND_LESS_THAN_SIGN        =  0x36,
    HID_KEYBOARD_SC_DOT_AND_GREATER_THAN_SIGN       =  0x37,
    HID_KEYBOARD_SC_SLASH_AND_QUESTION_MARK         =  0x38,
    HID_KEYBOARD_SC_CAPS_LOCK                       =  0x39,
    HID_KEYBOARD_SC_F1                              =  0x3A,
    HID_KEYBOARD_SC_F2                              =  0x3B,
    HID_KEYBOARD_SC_F3                              =  0x3C,
    HID_KEYBOARD_SC_F4                              =  0x3D,
    HID_KEYBOARD_SC_F5                              =  0x3E,
    HID_KEYBOARD_SC_F6                              =  0x3F,
    HID_KEYBOARD_SC_F7                              =  0x40,
    HID_KEYBOARD_SC_F8                              =  0x41,
    HID_KEYBOARD_SC_F9                              =  0x42,
    HID_KEYBOARD_SC_F10                             =  0x43,
    HID_KEYBOARD_SC_F11                             =  0x44,
    HID_KEYBOARD_SC_F12                             =  0x45,
    HID_KEYBOARD_SC_PRINT_SCREEN                    =  0x46,
    HID_KEYBOARD_SC_SCROLL_LOCK                     =  0x47,
    HID_KEYBOARD_SC_PAUSE                           =  0x48,
    HID_KEYBOARD_SC_INSERT                          =  0x49,
    HID_KEYBOARD_SC_HOME                            =  0x4A,
    HID_KEYBOARD_SC_PAGE_UP                         =  0x4B,
    HID_KEYBOARD_SC_DELETE                          =  0x4C,
    HID_KEYBOARD_SC_END                             =  0x4D,
    HID_KEYBOARD_SC_PAGE_DOWN                       =  0x4E,
    HID_KEYBOARD_SC_RIGHT_ARROW                     =  0x4F,
    HID_KEYBOARD_SC_LEFT_ARROW                      =  0x50,
    HID_KEYBOARD_SC_DOWN_ARROW                      =  0x51,
    HID_KEYBOARD_SC_UP_ARROW                        =  0x52,
    HID_KEYBOARD_SC_NUM_LOCK                        =  0x53,
    HID_KEYBOARD_SC_KEYPAD_SLASH                    =  0x54,
    HID_KEYBOARD_SC_KEYPAD_ASTERISK                 =  0x55,
    HID_KEYBOARD_SC_KEYPAD_MINUS                    =  0x56,
    HID_KEYBOARD_SC_KEYPAD_PLUS                     =  0x57,
    HID_KEYBOARD_SC_KEYPAD_ENTER                    =  0x58,
    HID_KEYBOARD_SC_KEYPAD_1_AND_END                =  0x59,
    HID_KEYBOARD_SC_KEYPAD_2_AND_DOWN_ARROW         =  0x5A,
    HID_KEYBOARD_SC_KEYPAD_3_AND_PAGE_DOWN          =  0x5B,
    HID_KEYBOARD_SC_KEYPAD_4_AND_LEFT_ARROW         =  0x5C,
    HID_KEYBOARD_SC_KEYPAD_5                        =  0x5D,
    HID_KEYBOARD_SC_KEYPAD_6_AND_RIGHT_ARROW        =  0x5E,
    HID_KEYBOARD_SC_KEYPAD_7_AND_HOME               =  0x5F,
    HID_KEYBOARD_SC_KEYPAD_8_AND_UP_ARROW           =  0x60,
    HID_KEYBOARD_SC_KEYPAD_9_AND_PAGE_UP            =  0x61,
    HID_KEYBOARD_SC_KEYPAD_0_AND_INSERT             =  0x62,
    HID_KEYBOARD_SC_KEYPAD_DOT_AND_DELETE           =  0x63,
    HID_KEYBOARD_SC_NON_US_BACKSLASH_AND_PIPE       =  0x64,
    HID_KEYBOARD_SC_APPLICATION                     =  0x65,
    HID_KEYBOARD_SC_POWER                           =  0x66,
    HID_KEYBOARD_SC_KEYPAD_EQUAL_SIGN               =  0x67,
    HID_KEYBOARD_SC_F13                             =  0x68,
    HID_KEYBOARD_SC_F14                             =  0x69,
    HID_KEYBOARD_SC_F15                             =  0x6A,
    HID_KEYBOARD_SC_F16                             =  0x6B,
    HID_KEYBOARD_SC_F17                             =  0x6C,
    HID_KEYBOARD_SC_F18                             =  0x6D,
    HID_KEYBOARD_SC_F19                             =  0x6E,
    HID_KEYBOARD_SC_F20                             =  0x6F,
    HID_KEYBOARD_SC_F21                             =  0x70,
    HID_KEYBOARD_SC_F22                             =  0x71,
    HID_KEYBOARD_SC_F23                             =  0x72,
    HID_KEYBOARD_SC_F24                             =  0x73,
    HID_KEYBOARD_SC_EXECUTE                         =  0x74,
    HID_KEYBOARD_SC_HELP                            =  0x75,
    HID_KEYBOARD_SC_MENU                            =  0x76,
    HID_KEYBOARD_SC_SELECT                          =  0x77,
    HID_KEYBOARD_SC_STOP                            =  0x78,
    HID_KEYBOARD_SC_AGAIN                           =  0x79,
    HID_KEYBOARD_SC_UNDO                            =  0x7A,
    HID_KEYBOARD_SC_CUT                             =  0x7B,
    HID_KEYBOARD_SC_COPY                            =  0x7C,
    HID_KEYBOARD_SC_PASTE                           =  0x7D,
    HID_KEYBOARD_SC_FIND                            =  0x7E,
    HID_KEYBOARD_SC_MUTE                            =  0x7F,
    HID_KEYBOARD_SC_VOLUME_UP                       =  0x80,
    HID_KEYBOARD_SC_VOLUME_DOWN                     =  0x81,
    HID_KEYBOARD_SC_LOCKING_CAPS_LOCK               =  0x82,
    HID_KEYBOARD_SC_LOCKING_NUM_LOCK                =  0x83,
    HID_KEYBOARD_SC_LOCKING_SCROLL_LOCK             =  0x84,
    HID_KEYBOARD_SC_KEYPAD_COMMA                    =  0x85,
    HID_KEYBOARD_SC_KEYPAD_EQUAL_SIGN_AS400         =  0x86,
    HID_KEYBOARD_SC_INTERNATIONAL1                  =  0x87,
    HID_KEYBOARD_SC_INTERNATIONAL2                  =  0x88,
    HID_KEYBOARD_SC_INTERNATIONAL3                  =  0x89,
    HID_KEYBOARD_SC_INTERNATIONAL4                  =  0x8A,
    HID_KEYBOARD_SC_INTERNATIONAL5                  =  0x8B,
    HID_KEYBOARD_SC_INTERNATIONAL6                  =  0x8C,
    HID_KEYBOARD_SC_INTERNATIONAL7                  =  0x8D,
    HID_KEYBOARD_SC_INTERNATIONAL8                  =  0x8E,
    HID_KEYBOARD_SC_INTERNATIONAL9                  =  0x8F,
    HID_KEYBOARD_SC_LANG1                           =  0x90,
    HID_KEYBOARD_SC_LANG2                           =  0x91,
    HID_KEYBOARD_SC_LANG3                           =  0x92,
    HID_KEYBOARD_SC_LANG4                           =  0x93,
    HID_KEYBOARD_SC_LANG5                           =  0x94,
    HID_KEYBOARD_SC_LANG6                           =  0x95,
    HID_KEYBOARD_SC_LANG7                           =  0x96,
    HID_KEYBOARD_SC_LANG8                           =  0x97,
    HID_KEYBOARD_SC_LANG9                           =  0x98,
    HID_KEYBOARD_SC_ALTERNATE_ERASE                 =  0x99,
    HID_KEYBOARD_SC_SYSREQ                          =  0x9A,
    HID_KEYBOARD_SC_CANCEL                          =  0x9B,
    HID_KEYBOARD_SC_CLEAR                           =  0x9C,
    HID_KEYBOARD_SC_PRIOR                           =  0x9D,
    HID_KEYBOARD_SC_RETURN                          =  0x9E,
    HID_KEYBOARD_SC_SEPARATOR                       =  0x9F,
    HID_KEYBOARD_SC_OUT                             =  0xA0,
    HID_KEYBOARD_SC_OPER                            =  0xA1,
    HID_KEYBOARD_SC_CLEAR_AND_AGAIN                 =  0xA2,
    HID_KEYBOARD_SC_CRSEL_AND_PROPS                 =  0xA3,
    HID_KEYBOARD_SC_EXSEL                           =  0xA4,
    HID_KEYBOARD_SC_KEYPAD_00                       =  0xB0,
    HID_KEYBOARD_SC_KEYPAD_000                      =  0xB1,
    HID_KEYBOARD_SC_THOUSANDS_SEPARATOR             =  0xB2,
    HID_KEYBOARD_SC_DECIMAL_SEPARATOR               =  0xB3,
    HID_KEYBOARD_SC_CURRENCY_UNIT                   =  0xB4,
    HID_KEYBOARD_SC_CURRENCY_SUB_UNIT               =  0xB5,
    HID_KEYBOARD_SC_KEYPAD_OPENING_PARENTHESIS      =  0xB6,
    HID_KEYBOARD_SC_KEYPAD_CLOSING_PARENTHESIS      =  0xB7,
    HID_KEYBOARD_SC_KEYPAD_OPENING_BRACE            =  0xB8,
    HID_KEYBOARD_SC_KEYPAD_CLOSING_BRACE            =  0xB9,
    HID_KEYBOARD_SC_KEYPAD_TAB                      =  0xBA,
    HID_KEYBOARD_SC_KEYPAD_BACKSPACE                =  0xBB,
    HID_KEYBOARD_SC_KEYPAD_A                        =  0xBC,
    HID_KEYBOARD_SC_KEYPAD_B                        =  0xBD,
    HID_KEYBOARD_SC_KEYPAD_C                        =  0xBE,
    HID_KEYBOARD_SC_KEYPAD_D                        =  0xBF,
    HID_KEYBOARD_SC_KEYPAD_E                        =  0xC0,
    HID_KEYBOARD_SC_KEYPAD_F                        =  0xC1,
    HID_KEYBOARD_SC_KEYPAD_XOR                      =  0xC2,
    HID_KEYBOARD_SC_KEYPAD_CARET                    =  0xC3,
    HID_KEYBOARD_SC_KEYPAD_PERCENTAGE               =  0xC4,
    HID_KEYBOARD_SC_KEYPAD_LESS_THAN_SIGN           =  0xC5,
    HID_KEYBOARD_SC_KEYPAD_GREATER_THAN_SIGN        =  0xC6,
    HID_KEYBOARD_SC_KEYPAD_AMP                      =  0xC7,
    HID_KEYBOARD_SC_KEYPAD_AMP_AMP                  =  0xC8,
    HID_KEYBOARD_SC_KEYPAD_PIPE                     =  0xC9,
    HID_KEYBOARD_SC_KEYPAD_PIPE_PIPE                =  0xCA,
    HID_KEYBOARD_SC_KEYPAD_COLON                    =  0xCB,
    HID_KEYBOARD_SC_KEYPAD_HASHMARK                 =  0xCC,
    HID_KEYBOARD_SC_KEYPAD_SPACE                    =  0xCD,
    HID_KEYBOARD_SC_KEYPAD_AT                       =  0xCE,
    HID_KEYBOARD_SC_KEYPAD_EXCLAMATION_SIGN         =  0xCF,
    HID_KEYBOARD_SC_KEYPAD_MEMORY_STORE             =  0xD0,
    HID_KEYBOARD_SC_KEYPAD_MEMORY_RECALL            =  0xD1,
    HID_KEYBOARD_SC_KEYPAD_MEMORY_CLEAR             =  0xD2,
    HID_KEYBOARD_SC_KEYPAD_MEMORY_ADD               =  0xD3,
    HID_KEYBOARD_SC_KEYPAD_MEMORY_SUBTRACT          =  0xD4,
    HID_KEYBOARD_SC_KEYPAD_MEMORY_MULTIPLY          =  0xD5,
    HID_KEYBOARD_SC_KEYPAD_MEMORY_DIVIDE            =  0xD6,
    HID_KEYBOARD_SC_KEYPAD_PLUS_AND_MINUS           =  0xD7,
    HID_KEYBOARD_SC_KEYPAD_CLEAR                    =  0xD8,
    HID_KEYBOARD_SC_KEYPAD_CLEAR_ENTRY              =  0xD9,
    HID_KEYBOARD_SC_KEYPAD_BINARY                   =  0xDA,
    HID_KEYBOARD_SC_KEYPAD_OCTAL                    =  0xDB,
    HID_KEYBOARD_SC_KEYPAD_DECIMAL                  =  0xDC,
    HID_KEYBOARD_SC_KEYPAD_HEXADECIMAL              =  0xDD,
    HID_KEYBOARD_SC_LEFT_CONTROL                    =  0xE0,
    HID_KEYBOARD_SC_LEFT_SHIFT                      =  0xE1,
    HID_KEYBOARD_SC_LEFT_ALT                        =  0xE2,
    HID_KEYBOARD_SC_LEFT_GUI                        =  0xE3,
    HID_KEYBOARD_SC_RIGHT_CONTROL                   =  0xE4,
    HID_KEYBOARD_SC_RIGHT_SHIFT                     =  0xE5,
    HID_KEYBOARD_SC_RIGHT_ALT                       =  0xE6,
    HID_KEYBOARD_SC_RIGHT_GUI                       =  0xE7,
    HID_KEYBOARD_SC_MEDIA_PLAY                      =  0xE8,
    HID_KEYBOARD_SC_MEDIA_STOP                      =  0xE9,
    HID_KEYBOARD_SC_MEDIA_PREVIOUS_TRACK            =  0xEA,
    HID_KEYBOARD_SC_MEDIA_NEXT_TRACK                =  0xEB,
    HID_KEYBOARD_SC_MEDIA_EJECT                     =  0xEC,
    HID_KEYBOARD_SC_MEDIA_VOLUME_UP                 =  0xED,
    HID_KEYBOARD_SC_MEDIA_VOLUME_DOWN               =  0xEE,
    HID_KEYBOARD_SC_MEDIA_MUTE                      =  0xEF,
    HID_KEYBOARD_SC_MEDIA_WWW                       =  0xF0,
    HID_KEYBOARD_SC_MEDIA_BACKWARD                  =  0xF1,
    HID_KEYBOARD_SC_MEDIA_FORWARD                   =  0xF2,
    HID_KEYBOARD_SC_MEDIA_CANCEL                    =  0xF3,
    HID_KEYBOARD_SC_MEDIA_SEARCH                    =  0xF4,
    HID_KEYBOARD_SC_MEDIA_SLEEP                     =  0xF8,
    HID_KEYBOARD_SC_MEDIA_LOCK                      =  0xF9,
    HID_KEYBOARD_SC_MEDIA_RELOAD                    =  0xFA,
    HID_KEYBOARD_SC_MEDIA_CALCULATOR                =  0xFB,
    HID_CSCP_HIDClass             = 0x03,
    HID_CSCP_NonBootSubclass      = 0x00,
    HID_CSCP_BootSubclass         = 0x01,
    HID_CSCP_NonBootProtocol      = 0x00,
    HID_CSCP_KeyboardBootProtocol = 0x01,
    HID_CSCP_MouseBootProtocol    = 0x02,
    HID_REQ_GetReport       = 0x01,
    HID_REQ_GetIdle         = 0x02,
    HID_REQ_GetProtocol     = 0x03,
    HID_REQ_SetReport       = 0x09,
    HID_REQ_SetIdle         = 0x0A,
    HID_REQ_SetProtocol     = 0x0B,
    HID_DTYPE_HID = 0x21,
    HID_DTYPE_Report = 0x22,
    HID_REPORT_ITEM_In      = 0,
    HID_REPORT_ITEM_Out     = 1,
    HID_REPORT_ITEM_Feature = 2;

struct USB_HID_Descriptor_HID_t
{
USB_Descriptor_Header_t Header; /**< Regular  descriptor's type and length. */
uint16_t                HIDSpec;
uint8_t CountryCode;
uint8_t TotalReportDescriptors;
uint8_t HIDReportType;
uint16_t HIDReportLength;
}
ATTR_PACKED;

		
struct USB_HID_StdDescriptor_HID_t
{
uint8_t  bLength; /**< Size of the descriptor, in bytes. */
uint8_t  bDescriptorType;
uint16_t bcdHID;
uint8_t  bCountryCode; /**< Country code of the localized device, or zero if universal. */
uint8_t  bNumDescriptors; /**< Total number of HID report descriptors for the interface. */
uint8_t  bDescriptorType2;
uint16_t wDescriptorLength;
} ATTR_PACKED;

struct USB_KeyboardReport_Data_t
{
    uint8_t Modifier;
    uint8_t Reserved;
    uint8_t KeyCode[6];
} ATTR_PACKED;

typedef uint8_t USB_Descriptor_HIDReport_Datatype_t;

typedef struct
{
    struct
    {
        uint8_t  InterfaceNumber;
        USB_Endpoint_Table_t ReportINEndpoint;
        void*    PrevReportINBuffer;
        uint8_t  PrevReportINBufferSize;
    } Config;

    struct
    {
        bool UsingReportProtocol;
        uint16_t PrevFrameNum;
        uint16_t IdleCount;
        uint16_t IdleMSRemaining;
    } State;
} USB_ClassInfo_HID_Device_t;

bool HID_Device_ConfigureEndpoints(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo);
void HID_Device_ProcessControlRequest(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo);
void HID_Device_USBTask(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo);

typedef struct
{
    USB_Descriptor_Configuration_Header_t Config;
    USB_Descriptor_Interface_t            HID_Interface;
    USB_HID_Descriptor_HID_t              HID_KeyboardHID;
    USB_Descriptor_Endpoint_t             HID_ReportINEndpoint;
    USB_Descriptor_Endpoint_t             HID_ReportOUTEndpoint;
} USB_Descriptor_Configuration_t;

enum InterfaceDescriptors_t
{
    INTERFACE_ID_Keyboard = 0, /**< Keyboard interface descriptor ID */
};

enum StringDescriptors_t
{
    STRING_ID_Language     = 0, /**< Supported Languages string descriptor ID (must be zero) */
    STRING_ID_Manufacturer = 1, /**< Manufacturer string ID */
    STRING_ID_Product      = 2, /**< Product string ID */
};

#define KEYBOARD_IN_EPADDR        (ENDPOINT_DIR_IN  | 1)
#define KEYBOARD_OUT_EPADDR       (ENDPOINT_DIR_OUT | 2)

uint16_t getDescriptor(const uint16_t wValue,
                     const uint16_t wIndex, const void** const DescriptorAddress);


class USBKB
{
private:
    inline uint8_t read8() const { return UEDATX; }
    inline void write8(uint8_t dat) { UEDATX = dat; }
    inline void write16(uint16_t dat) { UEDATX = dat & 0xff; UEDATX = dat >> 8; }
    volatile bool USB_IsInitialized;
    uint8_t USB_Device_ConfigurationNumber;
    bool USB_Device_CurrentlySelfPowered;
    bool USB_Device_RemoteWakeupEnabled;
    USB_Request_Header_t ctrlReq;
    volatile uint8_t USB_DeviceState;
    uint8_t writeStream(const void * const buf, uint16_t len, uint16_t* const bytes);
    uint8_t Endpoint_Read_Stream_LE (void * const buf, uint16_t len, uint16_t* const bytes);
    uint8_t Endpoint_Read_Control_Stream_LE(void* const Buffer, uint16_t Length);
    uint8_t Endpoint_WaitUntilReady();
    uint8_t Endpoint_Write_Control_PStream_LE (const void* const Buffer, uint16_t Length);
    void USB_Device_GetDescriptor();
    void USB_Device_ProcessControlRequest();
    void Endpoint_ClearStatusStage();
    void Device_ClearSetFeature();
    uint8_t Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length);
    void USB_Device_GetInternalSerialDescriptor();
    bool Endpoint_ConfigureEndpoint_Prv(uint8_t Number, uint8_t UECFG0XData, uint8_t UECFG1XData);
    bool Endpoint_ConfigureEndpoint(uint8_t Address, uint8_t Type, uint16_t Size, uint8_t Banks);
    bool Endpoint_ConfigureEndpointTable(const USB_Endpoint_Table_t* const Table, uint8_t Entries);
public:
    static USBKB *instance;
    void com();
    void gen();
    USBKB();
    void hidtask();
};

USBKB *USBKB::instance;

bool USBKB::Endpoint_ConfigureEndpoint(uint8_t Address, uint8_t Type, uint16_t Size, uint8_t Banks)
{
    uint8_t Number = (Address & ENDPOINT_EPNUM_MASK);

    if (Number >= ENDPOINT_TOTAL_ENDPOINTS)
        return false;

    return Endpoint_ConfigureEndpoint_Prv(Number,
          ((Type << EPTYPE0) | ((Address & ENDPOINT_DIR_IN) ? (1 << EPDIR) : 0)),
          ((1 << ALLOC) | ((Banks > 1) ? (1 << EPBK0) : 0) | Endpoint_BytesToEPSizeMask(Size)));
}

void USBKB::hidtask()
{
    if (USB_DeviceState != DEVICE_STATE_Configured)
        return;

    static USB_KeyboardReport_Data_t PrevKeyboardReportData;
    USB_KeyboardReport_Data_t        KeyboardReportData;
    bool                             SendReport = false;
    uint8_t UsedKeyCodes      = 0;
    memset(&KeyboardReportData, 0, sizeof(USB_KeyboardReport_Data_t));

    if ((PINF & 1<<0) == 0)
        KeyboardReportData.KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_A;

    if ((PINF & 1<<1) == 0)
        KeyboardReportData.KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_B;

    if ((PINF & 1<<4) == 0)
        KeyboardReportData.KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_C;

    if ((PINF & 1<<5) == 0)
        KeyboardReportData.KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_D;

    if ((PINF & 1<<6) == 0)
        KeyboardReportData.KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_E;

    if ((PINF & 1<<7) == 0)
        KeyboardReportData.KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_F;

    Endpoint_SelectEndpoint(KEYBOARD_IN_EPADDR);
    PrevKeyboardReportData = KeyboardReportData;
    writeStream(&KeyboardReportData, sizeof(KeyboardReportData), NULL);
    UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
    Endpoint_SelectEndpoint(KEYBOARD_OUT_EPADDR);

    if (UEINTX & 1<<RXOUTI)
    {
        if (UEINTX & 1<<RWAL)
            read8();

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }
}

USBKB::USBKB()
{
    instance = this;
	clock_prescale_set(clock_div_2);
    UHWCON |= 1<<UVREGE;
    USB_IsInitialized = true;
    USBCON &= ~(1<<VBUSTE);
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1<<USBE);       // disable usb controller
    USBCON |= 1<<USBE;          // enable usb controller
    USBCON &= ~(1<<FRZCLK);
    PLLCSR = 0;
    USB_DeviceState                 = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    UDCON &= ~(1<<LSM);         // full speed
    USBCON |= 1<<VBUSTE;        // enable vbus int
    Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL, 8, 1);
    UDINT &= ~(1<<SUSPI);
    UDIEN |= 1<<SUSPE;
    UDIEN |= 1<<EORSTE;
    UDCON &= ~(1<<DETACH);
    USBCON |= 1<<OTGPADE;
	sei();
}

int main(void)
{
    DDRF &= ~(1<<0 | 1<<1 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    PORTF |= 1<<0 | 1<<1 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
    USBKB kb;

	while (true)
	{
        kb.hidtask();
	}
}

void USBKB::Endpoint_ClearStatusStage()
{
    if (ctrlReq.bmRequestType & REQDIR_DEVICETOHOST)
    {
        while (!(UEINTX & 1<<RXOUTI))
            if (USB_DeviceState == DEVICE_STATE_Unattached)
                return;

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }
    else
    {
        while (!(Endpoint_IsINReady()))
        {
            if (USB_DeviceState == DEVICE_STATE_Unattached)
              return;
        }

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN
    }
}

uint8_t USBKB::Endpoint_WaitUntilReady()
{
    uint8_t  TimeoutMSRem = USB_STREAM_TIMEOUT_MS;
    uint16_t PreviousFrameNumber = UDFNUM;

    while (true)
    {
        if (Endpoint_GetEndpointDirection() == ENDPOINT_DIR_IN)
        {
            if (Endpoint_IsINReady())
                return ENDPOINT_READYWAIT_NoError;
        }
        else
        {
            if (UEINTX & 1<<RXOUTI)
                return ENDPOINT_READYWAIT_NoError;
        }

        uint8_t USB_DeviceState_LCL = USB_DeviceState;

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

bool USBKB::Endpoint_ConfigureEndpoint_Prv(uint8_t Number,
    uint8_t UECFG0XData, uint8_t UECFG1XData)
{
    for (uint8_t EPNum = Number; EPNum < ENDPOINT_TOTAL_ENDPOINTS; EPNum++)
    {
        uint8_t UECFG0XTemp, UECFG1XTemp, UEIENXTemp;
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

        if (!(UECFG1XTemp & 1<<ALLOC))
            continue;

        UECONX &= ~(1<<EPEN);
        UECFG1X &= ~(1<<ALLOC);
        UECONX |= 1<<EPEN;
        UECFG0X = UECFG0XTemp;
        UECFG1X = UECFG1XTemp;
        UEIENX  = UEIENXTemp;

        if ((UESTA0X & 1<<CFGOK) == 0)
            return false;
    }

    Endpoint_SelectEndpoint(Number);
    return true;
}

void USBKB::USB_Device_GetInternalSerialDescriptor()
{
    struct
    {
        USB_Descriptor_Header_t Header;
        uint16_t                UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
    } SignatureDescriptor;

    SignatureDescriptor.Header.Type = DTYPE_String;
    SignatureDescriptor.Header.Size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
    USB_Device_GetSerialString(SignatureDescriptor.UnicodeString);
    UEINTX &= ~(1<<RXSTPI);
    Endpoint_Write_Control_Stream_LE(&SignatureDescriptor, sizeof(SignatureDescriptor));
    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
}

bool USBKB::Endpoint_ConfigureEndpointTable(const USB_Endpoint_Table_t* const Table,
                                     uint8_t Entries)
{
    for (uint8_t i = 0; i < Entries; i++)
    {
        if (!(Table[i].Address))
          continue;

        if (!(Endpoint_ConfigureEndpoint(Table[i].Address, Table[i].Type, Table[i].Size, Table[i].Banks)))
          return false;
    }

    return true;
}

void USBKB::USB_Device_GetDescriptor()
{
    const void *descPtr;
    uint16_t descSize;

    if (ctrlReq.wValue == ((DTYPE_String << 8) | USE_INTERNAL_SERIAL))
    {
        USB_Device_GetInternalSerialDescriptor();
        return;
    }

    if ((descSize = getDescriptor(ctrlReq.wValue, ctrlReq.wIndex, &descPtr)) == NO_DESCRIPTOR)
    {
        return;
    }

    UEINTX &= ~(1<<RXSTPI);
    Endpoint_Write_Control_PStream_LE(descPtr, descSize);
    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
}

void USBKB::Device_ClearSetFeature()
{
    switch (ctrlReq.bmRequestType & CONTROL_REQTYPE_RECIPIENT)
    {
        case REQREC_DEVICE:
            if ((uint8_t)ctrlReq.wValue == FEATURE_SEL_DeviceRemoteWakeup)
                USB_Device_RemoteWakeupEnabled = (ctrlReq.bRequest == REQ_SetFeature);
            else
                return;

            break;
        case REQREC_ENDPOINT:
            if ((uint8_t)ctrlReq.wValue == FEATURE_SEL_EndpointHalt)
            {
                uint8_t index = ((uint8_t)ctrlReq.wIndex & ENDPOINT_EPNUM_MASK);

                if (index == ENDPOINT_CONTROLEP || index >= ENDPOINT_TOTAL_ENDPOINTS)
                    return;

                Endpoint_SelectEndpoint(index);

                if (UECONX & 1<<EPEN)
                {
                    if (ctrlReq.bRequest == REQ_SetFeature)
                    {
                        UECONX |= 1<<STALLRQ;   // stall transaction
                    }
                    else
                    {
                        UECONX |= 1<<STALLRQC;  // clear stall
                        Endpoint_ResetEndpoint(index);
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

void USBKB::USB_Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&ctrlReq;

    for (uint8_t i = 0; i < sizeof(USB_Request_Header_t); i++)
        *(RequestHeader++) = read8();


    if (UEINTX & 1<<RXSTPI) // is setup received?
    {
        uint8_t bmRequestType = ctrlReq.bmRequestType;

        switch (ctrlReq.bRequest)
        {
        case REQ_GetStatus:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                uint8_t CurrentStatus = 0;

                switch (ctrlReq.bmRequestType)
                {
                case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE):
                    if (USB_Device_CurrentlySelfPowered)
                        CurrentStatus |= FEATURE_SELFPOWERED_ENABLED;

                    if (USB_Device_RemoteWakeupEnabled)
                        CurrentStatus |= FEATURE_REMOTE_WAKEUP_ENABLED;

                    break;
                case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT):
                {
                   uint8_t index = ((uint8_t)ctrlReq.wIndex & ENDPOINT_EPNUM_MASK);

                    if (index >= ENDPOINT_TOTAL_ENDPOINTS)
                        return;

                    Endpoint_SelectEndpoint(index);
                    CurrentStatus = UECONX & 1<<STALLRQ;
                    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
                }
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
                uint8_t devAddr = ctrlReq.wValue & 0x7F;
                USB_Device_SetDeviceAddress(devAddr);
                UEINTX &= ~(1<<RXSTPI);
                Endpoint_ClearStatusStage();
                while (!(Endpoint_IsINReady()));
                UDADDR |= 1<<ADDEN;
                USB_DeviceState = devAddr ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
            }
            break;
        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                USB_Device_GetDescriptor();
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
                if ((uint8_t)ctrlReq.wValue > 1)
                    return;

                UEINTX &= ~(1<<RXSTPI);
                USB_Device_ConfigurationNumber = (uint8_t)ctrlReq.wValue;
                Endpoint_ClearStatusStage();

                if (USB_Device_ConfigurationNumber)
                {
                    USB_DeviceState = DEVICE_STATE_Configured;
                }
                else
                {
                    USB_DeviceState = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured :
                        DEVICE_STATE_Powered;
                }

                Endpoint_ConfigureEndpoint(ENDPOINT_DIR_IN | 1, EP_TYPE_INTERRUPT, 8, 1);
                Endpoint_ConfigureEndpoint(ENDPOINT_DIR_OUT | 2, EP_TYPE_INTERRUPT, 8, 1);
                UDIEN |= 1<<SOFE;
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

ISR(USB_COM_vect, ISR_BLOCK)
{
    USBKB::instance->com();
}

void USBKB::com()
{
    uint8_t PrevSelectedEndpoint = Endpoint_GetCurrentEndpoint();
    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    UEIENX &= ~(1<<RXSTPE);
    sei();
    USB_Device_ProcessControlRequest();
    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    UEIENX |= 1<<RXSTPE;
    Endpoint_SelectEndpoint(PrevSelectedEndpoint);
}

ISR(USB_GEN_vect, ISR_BLOCK)
{
    USBKB::instance->gen();
}

void USBKB::gen()
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE)
    {
        UDINT &= ~(1<<SOFI);    // clear INT SOFI
    }

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            PLLCSR = 1<<PINDIV;
            PLLCSR = 1<<PINDIV | 1<<PLLE;

            while ((PLLCSR & 1<<PLOCK) == 0)
                ;

            USB_DeviceState = DEVICE_STATE_Powered;
        }
        else
        {
            PLLCSR = 0;
            USB_DeviceState = DEVICE_STATE_Unattached;
        }
    }

    if (UDINT & 1<<SUSPI && UDIEN & 1<<SUSPE)
    {
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        USBCON |= 1<<FRZCLK;
        PLLCSR = 0;
        USB_DeviceState = DEVICE_STATE_Suspended;
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)
    {
        PLLCSR = 1<<PINDIV;
        PLLCSR = 1<<PINDIV | 1<<PLLE;
        while ((PLLCSR & 1<<PLOCK) == 0);
        USBCON &= ~(1 << FRZCLK);
        UDINT &= ~(1<<WAKEUPI);     // int clear
        UDIEN &= ~(1<<WAKEUPI);     // int disable
        UDIEN |= 1<<SUSPE;

        if (USB_Device_ConfigurationNumber)
            USB_DeviceState = DEVICE_STATE_Configured;
        else
            USB_DeviceState = UDADDR & 1<<ADDEN ? DEVICE_STATE_Addressed : DEVICE_STATE_Powered;

    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);
        USB_DeviceState = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL, 8, 1);
        UEIENX |= 1<<RXSTPE;
    }
}

uint8_t USBKB::writeStream(const void * const Buffer, uint16_t Length,
                            uint16_t* const BytesProcessed)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
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
        if ((UEINTX & 1<<RWAL) == 0)
        {
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

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
            write8(*DataStream);
            DataStream += 1;
            Length--;
            BytesInTransfer++;
        }
    }

    return ENDPOINT_RWSTREAM_NoError;
}

uint8_t USBKB::Endpoint_Read_Stream_LE(void * const Buffer, uint16_t Length,
                            uint16_t* const BytesProcessed)
{
    uint8_t* DataStream      = ((uint8_t*)Buffer);
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
        if ((UEINTX & 1<<RWAL) == 0)
        {
            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);

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
            *DataStream = read8();
            DataStream += 1;
            Length--;
            BytesInTransfer++;
        }
    }

    return ENDPOINT_RWSTREAM_NoError;
}

uint8_t USBKB::Endpoint_Read_Control_Stream_LE(void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream = ((uint8_t*)Buffer);

    if (!(Length))
        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);

    while (Length)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOINT_RWCSTREAM_HostAborted;

        if (UEINTX & 1<<RXOUTI)
        {
            while (Length && Endpoint_BytesInEndpoint())
            {
                *DataStream = read8();
                DataStream += 1;
                Length--;
            }

            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
        }
    }

    while (!(Endpoint_IsINReady()))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t USBKB::Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = ((uint8_t*)Buffer);
    bool     LastPacketFull = false;

    if (Length > ctrlReq.wLength)
        Length = ctrlReq.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
          return ENDPOINT_RWCSTREAM_HostAborted;
        else if (UEINTX & 1<<RXOUTI)
          break;

        if (Endpoint_IsINReady())
        {
            uint16_t BytesInEndpoint = Endpoint_BytesInEndpoint();

            while (Length && (BytesInEndpoint < 8))
            {
                write8(*DataStream);
                DataStream += 1;
                Length--;
                BytesInEndpoint++;
            }
            
            LastPacketFull = (BytesInEndpoint == 8);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN
        }
    }

    while (!(UEINTX & 1<<RXOUTI))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOINT_RWCSTREAM_HostAborted;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t USBKB::Endpoint_Write_Control_PStream_LE (const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = (uint8_t*)Buffer;
    bool     LastPacketFull = false;

    if (Length > ctrlReq.wLength)
        Length = ctrlReq.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOINT_RWCSTREAM_HostAborted;
        else if (UEINTX & 1<<RXOUTI)
            break;

        if (Endpoint_IsINReady())
        {
            uint16_t BytesInEndpoint = Endpoint_BytesInEndpoint();

            while (Length && (BytesInEndpoint < 8))
            {
                write8(pgm_read_byte(DataStream));
                DataStream += 1;
                Length--;
                BytesInEndpoint++;
            }
            
            LastPacketFull = (BytesInEndpoint == 8);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
        }
    }

    while (!(UEINTX & 1<<RXOUTI))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOINT_RWCSTREAM_HostAborted;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}



const uint8_t PROGMEM KeyboardReport[] =
{
    0x04 | 0x00 | 1, 0x01,
    0x08 | 0x00 | 1, 0x06,
    0x00 | 0xA0 | 1, 0x01,
    0x04 | 0x00 | 1, 0x07,
    0x08 | 0x10 | 1, 0xE0,
    0x08 | 0x20 | 1, 0xE7,
    0x04 | 0x10 | 1, 0x00,
    0x04 | 0x20 | 1, 0x01,
    0x04 | 0x70 | 1, 0x01,
    0x04 | 0x90 | 1, 0x08,
    0x00 | 0x80 | 1, 1<<1,
    0x04 | 0x90 | 1, 0x01,
    0x04 | 0x70 | 1, 0x08,
    0x00 | 0x80 | 1, 1<<0,
    0x04 | 0x00 | 1, 0x08,
    0x08 | 0x10 | 1, 0x01,
    0x08 | 0x20 | 1, 0x05,
    0x04 | 0x90 | 1, 0x05,
    0x04 | 0x70 | 1, 0x01,
    0x00 | 0x90 | 1, 1<<1,
    0x04 | 0x90 | 1, 0x01,
    0x04 | 0x70 | 1, 0x03,
    0x00 | 0x90 | 1, 1<<0,
    0x04 | 0x10 | 1, 0x00,
    0x04 | 0x20 | 1, 0x65,
    0x04 | 0x00 | 1, 0x07,
    0x08 | 0x10 | 1, 0x00,
    0x08 | 0x20 | 1, 0x65,
    0x04 | 0x90 | 1, 0x06,
    0x04 | 0x70 | 1, 0x08,
    0x00 | 0x80 | 1, 0,
    0x00 | 0xC0 | 0
};

const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
    {
        sizeof(USB_Descriptor_Device_t),
        DTYPE_Device
    },

	0x0110,
	USB_CSCP_NoDeviceClass,
	USB_CSCP_NoDeviceSubclass,
	USB_CSCP_NoDeviceProtocol,
    8,
	0x03EB,
	0x2042,
	0x0001,
	STRING_ID_Manufacturer,
	STRING_ID_Product,
	NO_DESCRIPTOR,
    1
};

const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
    {
        {
            sizeof(USB_Descriptor_Configuration_Header_t),
            DTYPE_Configuration
        },

        sizeof(USB_Descriptor_Configuration_t),
        1,
        1,
        NO_DESCRIPTOR,
        USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED,
        USB_CONFIG_POWER_MA(100)
    },
    {   // HID_Interface
        {
            sizeof(USB_Descriptor_Interface_t),
            DTYPE_Interface
        },
        INTERFACE_ID_Keyboard,
        0,
        2,
        HID_CSCP_HIDClass,
        HID_CSCP_BootSubclass,
        HID_CSCP_KeyboardBootProtocol,
        NO_DESCRIPTOR
    },
    {   // HID_KeyboardHID
        .Header =
        {
            sizeof(USB_HID_Descriptor_HID_t),
            HID_DTYPE_HID
        },
        0x0111,
        0x00,
        1,
        HID_DTYPE_Report,
        sizeof(KeyboardReport)
    },
    {   // HID_ReportINEndpoint
        {
            sizeof(USB_Descriptor_Endpoint_t),
            DTYPE_Endpoint
        },

        KEYBOARD_IN_EPADDR,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        8,
        5
    },
    {
        {
            sizeof(USB_Descriptor_Endpoint_t),
            DTYPE_Endpoint
        },

        KEYBOARD_OUT_EPADDR,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        8,
        5
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

uint16_t getDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

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
				case STRING_ID_Language:
					Address = &LanguageString;
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;
				case STRING_ID_Manufacturer:
					Address = &ManufacturerString;
					Size    = pgm_read_byte(&ManufacturerString.Header.Size);
					break;
				case STRING_ID_Product:
					Address = &ProductString;
					Size    = pgm_read_byte(&ProductString.Header.Size);
					break;
			}

			break;
		case HID_DTYPE_HID:
			Address = &ConfigurationDescriptor.HID_KeyboardHID;
			Size    = sizeof(USB_HID_Descriptor_HID_t);
			break;
		case HID_DTYPE_Report:
			Address = &KeyboardReport;
			Size    = sizeof(KeyboardReport);
			break;
	}

	*DescriptorAddress = Address;
	return Size;
}

