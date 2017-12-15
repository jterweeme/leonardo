#include "usbkb2.h"
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include <avr/power.h>

static constexpr uint8_t
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
    HID_DTYPE_HID           = 0x21,
    HID_DTYPE_Report        = 0x22,
    HID_REPORT_ITEM_In      = 0,
    HID_REPORT_ITEM_Out     = 1,
    HID_REPORT_ITEM_Feature = 2;

struct USB_Descriptor_Header_t
{
    uint8_t Size;
    uint8_t Type;
}
__attribute__ ((packed));

struct USB_HID_Descriptor_HID_t
{
    USB_Descriptor_Header_t Header;
    uint16_t HIDSpec;
    uint8_t CountryCode;
    uint8_t TotalReportDescriptors;
    uint8_t HIDReportType;
    uint16_t HIDReportLength;
}
__attribute__ ((packed));


template <size_t S> struct USB_Descriptor_String_t
{
    USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
    wchar_t UnicodeString[S];
}
__attribute__ ((packed));


struct USB_Descriptor_Endpoint_t
{
    USB_Descriptor_Header_t Header;
    uint8_t  EndpointAddress;
    uint8_t  Attributes;
    uint16_t EndpointSize;
    uint8_t  PollingIntervalMS;
}
__attribute__ ((packed));


struct USB_Descriptor_Interface_Association_t
{
    USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
    uint8_t FirstInterfaceIndex; /**< Index of the first associated interface. */
    uint8_t TotalInterfaces; /**< Total number of associated interfaces. */
    uint8_t Class; /**< Interface class ID. */
    uint8_t SubClass; /**< Interface subclass ID. */
    uint8_t Protocol; /**< Interface protocol ID. */
    uint8_t IADStrIndex;
}
__attribute__ ((packed));


struct USB_Descriptor_Interface_t
{
    USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
    uint8_t InterfaceNumber; /**< Index of the interface in the current configuration. */
    uint8_t AlternateSetting;
    uint8_t TotalEndpoints; /**< Total number of endpoints in the interface. */
    uint8_t Class; /**< Interface class ID. */
    uint8_t SubClass; /**< Interface subclass ID. */
    uint8_t Protocol; /**< Interface protocol ID. */
    uint8_t InterfaceStrIndex;
}
__attribute__ ((packed));

struct USB_Descriptor_DeviceQualifier_t
{
    USB_Descriptor_Header_t Header;
    uint16_t USBSpecification;
    uint8_t  Class; /**< USB device class. */
    uint8_t  SubClass; /**< USB device subclass. */
    uint8_t  Protocol; /**< USB device protocol. */
    uint8_t  Endpoint0Size;
    uint8_t  NumberOfConfigurations;
    uint8_t  Reserved; /**< Reserved for future use, must be 0. */
}
__attribute__ ((packed));

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
} 
__attribute__ ((packed));

struct USB_Descriptor_Configuration_Header_t
{
    USB_Descriptor_Header_t Header;
    uint16_t TotalConfigurationSize;
    uint8_t  TotalInterfaces;
    uint8_t  ConfigurationNumber;
    uint8_t  ConfigurationStrIndex;
    uint8_t  ConfigAttributes;
    uint8_t  MaxPowerConsumption;
}
__attribute__ ((packed));

struct USB_Descriptor_Configuration_t
{
    USB_Descriptor_Configuration_Header_t Config;
    USB_Descriptor_Interface_t            HID_Interface;
    USB_HID_Descriptor_HID_t              HID_KeyboardHID;
    USB_Descriptor_Endpoint_t             HID_ReportINEndpoint;
    USB_Descriptor_Endpoint_t             HID_ReportOUTEndpoint;
};

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

const uint8_t PROGMEM KeyboardReport[] =
{
    0x04 | 0x00 | 0x01, 0x01 & 0xFF,
    0x08 | 0x00 | 0x01, 0x06 & 0xFF,
    0x00 | 0xA0 | 0x01, 0x01 & 0xFF,
    0x04 | 0x00 | 0x01, 0x07 & 0xFF,
    0x08 | 0x10 | 0x01, 0xE0 & 0xFF,
    0x08 | 0x20 | 0x01, 0xE7 & 0xFF,
    0x04 | 0x10 | 0x01, 0x00 & 0xFF,
    0x04 | 0x20 | 0x01, 0x01 & 0xFF,
    0x04 | 0x70 | 0x01, 0x01 & 0xFF,
    0x04 | 0x90 | 0x01, 0x08 & 0xFF,
    0x00 | 0x80 | 0x01, ((0<<0 | 1<<1 | 0<<2) & 0xFF),
    0x04 | 0x90 | 0x01, ((0x01) & 0xFF),
    0x04 | 0x70 | 0x01, ((0x08) & 0xFF),
    0x00 | 0x80 | 0x01, (((1 << 0)) & 0xFF),
    0x04 | 0x00 | 0x01, 0x08 & 0xFF,
    0x08 | 0x10 | 0x01, 0x01 & 0xFF,
    0x08 | 0x20 | 0x01, 0x05 & 0xFF,
    0x04 | 0x90 | 0x01, 0x05 & 0xFF,
    0x04 | 0x70 | 0x01, 0x01 & 0xFF,
    0x00 | 0x90 | 0x01, (((0 << 0) | (1 << 1) | (0 << 2) | (0 << 7)) & 0xFF),
    0x04 | 0x90 | 0x01, ((0x01) & 0xFF),
    0x04 | 0x70 | 0x01, ((0x03) & 0xFF),
    0x00 | 0x90 | 0x01, (((1 << 0)) & 0xFF),
    0x04 | 0x10 | 0x01, ((0x00) & 0xFF),
    0x04 | 0x20 | 0x01, ((0x65) & 0xFF),
    0x04 | 0x00 | 0x01, ((0x07) & 0xFF),
    0x08 | 0x10 | 0x01, ((0x00) & 0xFF),
    0x08 | 0x20 | 0x01, ((0x65) & 0xFF),
    0x04 | 0x90 | 0x01, ((0x06) & 0xFF),
    0x04 | 0x70 | 0x01, ((0x08) & 0xFF),
    0x00 | 0x80 | 0x01, (((0 << 0) | (0 << 1) | (0 << 2)) & 0xFF),
    0x00 | 0xC0 | 0x00
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

#define USB_STRING_LEN(UnicodeChars) (sizeof(USB_Descriptor_Header_t) + ((UnicodeChars) << 1))

#define USB_STRING_DESCRIPTOR(String)     { .Header = {.Size = sizeof(USB_Descriptor_Header_t) + (sizeof(String) - 2), .Type = DTYPE_String}, .UnicodeString = String }


#define USB_CONFIG_ATTR_RESERVED          0x80
#define USB_CONFIG_ATTR_SELFPOWERED       0x40
#define USB_CONFIG_ATTR_REMOTEWAKEUP      0x20
#define ENDPOINT_ATTR_NO_SYNC             (0 << 2)
#define ENDPOINT_ATTR_ASYNC               (1 << 2)
#define ENDPOINT_ATTR_ADAPTIVE            (2 << 2)
#define ENDPOINT_ATTR_SYNC                (3 << 2)
#define ENDPOINT_USAGE_DATA               (0 << 4)
#define ENDPOINT_USAGE_FEEDBACK           (1 << 4)
#define ENDPOINT_USAGE_IMPLICIT_FEEDBACK  (2 << 4)
#define USE_INTERNAL_SERIAL            0xDC
#define INTERNAL_SERIAL_LENGTH_BITS    80
#define INTERNAL_SERIAL_START_ADDRESS  0x0E


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
        {
            sizeof(USB_HID_Descriptor_HID_t),
            HID_DTYPE_HID
        },
        0x0111,
        0,
        1,
        HID_DTYPE_Report,
        sizeof(KeyboardReport)
    },
    {   // HID_ReportINEndpoint
        {
            sizeof(USB_Descriptor_Endpoint_t),
            DTYPE_Endpoint
        },

        ENDPOINT_DIR_IN  | 1,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        8,
        5
    },
    {   // HID_ReportOUTEndpoint
        {
            sizeof(USB_Descriptor_Endpoint_t),
            DTYPE_Endpoint
        },

        ENDPOINT_DIR_OUT | 2,
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

uint16_t USBKB::CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
    const uint8_t DescriptorType = wValue >> 8;
    const uint8_t DescriptorNumber = wValue & 0xFF;
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
                case STRING_ID_Language:
                    Address = &LanguageString;
                    Size = pgm_read_byte(&LanguageString.Header.Size);
                    break;
                case STRING_ID_Manufacturer:
                    Address = &ManufacturerString;
                    Size = pgm_read_byte(&ManufacturerString.Header.Size);
                    break;
                case STRING_ID_Product:
                    Address = &ProductString;
                    Size = pgm_read_byte(&ProductString.Header.Size);
                    break;
            }

            break;
        case HID_DTYPE_HID:
            Address = &ConfigurationDescriptor.HID_KeyboardHID;
            Size = sizeof(USB_HID_Descriptor_HID_t);
            break;
        case HID_DTYPE_Report:
            Address = &KeyboardReport;
            Size = sizeof(KeyboardReport);
            break;
    }

    *DescriptorAddress = Address;
    return Size;
}

static inline uint8_t GetGlobalInterruptMask(void)
{
    __asm__ __volatile__("" ::: "memory");
    return SREG;
}

static inline void USB_Device_GetSerialString(uint16_t* const UnicodeString)
{
    uint8_t CurrentGlobalInt = GetGlobalInterruptMask();
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

        UnicodeString[SerialCharNum] = (SerialByte >= 10) ?
            (('A' - 10) + SerialByte) : ('0' + SerialByte);
    }

    __asm__ __volatile__("" ::: "memory");
    SREG = CurrentGlobalInt;
    __asm__ __volatile__("" ::: "memory");
}

uint8_t USBKB::Endpoint_Write_Stream_LE (const void * const Buffer, uint16_t Length,
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
        if (!(UEINTX & 1<<RWAL))
        {
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();
            selectEndpoint(ENDPOINT_CONTROLEP);

            if (UEINTX & 1<<RXSTPI)     // is setup received?
                USB_Device_ProcessControlRequest();

            selectEndpoint(PrevEndpoint);

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

uint8_t USBKB::Endpoint_Write_Control_PStream_LE (const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = (uint8_t*)Buffer;
    bool     LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = state;

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
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOINT_RWCSTREAM_HostAborted;
    }

    return ENDPOINT_RWCSTREAM_NoError;
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
    state = DEVICE_STATE_Unattached;
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

void USBKB::Device_ClearSetFeature()
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
                uint8_t index = ((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);

                if (index == ENDPOINT_CONTROLEP || index >= ENDPOINT_TOTAL_ENDPOINTS)
                    return;

                selectEndpoint(index);

                if (UECONX & 1<<EPEN)
                {
                    if (USB_ControlRequest.bRequest == REQ_SetFeature)
                    {
                        UECONX |= 1<<STALLRQ;   // stall transaction
                    }
                    else
                    {
                        UECONX |= 1<<STALLRQC;  // clear stall
                        UERST = 1 << (index & ENDPOINT_EPNUM_MASK);
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

uint8_t USBKB::Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = ((uint8_t*)Buffer);
    bool     LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = state;

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
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
          return ENDPOINT_RWCSTREAM_HostAborted;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

void USBKB::com()
{
    uint8_t PrevSelectedEndpoint = Endpoint_GetCurrentEndpoint();
    selectEndpoint(ENDPOINT_CONTROLEP);
    UEIENX &= ~(1<<RXSTPE);
    sei();
    USB_Device_ProcessControlRequest();
    selectEndpoint(ENDPOINT_CONTROLEP);
    UEIENX |= 1<<RXSTPE;
    selectEndpoint(PrevSelectedEndpoint);
}

void USBKB::gen()
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE)
    {
        UDINT &= ~(1<<SOFI);    // clear INT SOFI

        if (IdleMSRemaining)
            IdleMSRemaining--;
    }

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            PLLCSR = 1<<PINDIV;
            PLLCSR = 1<<PINDIV | 1<<PLLE;
            while (!(PLLCSR & 1<<PLOCK));
            state = DEVICE_STATE_Powered;
        }
        else
        {
            PLLCSR = 0;
            state = DEVICE_STATE_Unattached;
        }
    }

    if (UDINT & 1<<SUSPI && UDIEN & 1<<SUSPE)
    {
        UDIEN  &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        USBCON |= 1<<FRZCLK;
        PLLCSR = 0;
        state = DEVICE_STATE_Suspended;
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)
    {
        PLLCSR = 1<<PINDIV;
        PLLCSR = 1<<PINDIV | 1<<PLLE;
        while (!(PLLCSR & 1<<PLOCK));
        USBCON &= ~(1 << FRZCLK);
        UDINT &= ~(1<<WAKEUPI);     // int clear
        UDIEN &= ~(1<<WAKEUPI);     // int disable
        UDIEN |= 1<<SUSPE;

        if (USB_Device_ConfigurationNumber)
            state = DEVICE_STATE_Configured;
        else
            state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Addressed : DEVICE_STATE_Powered;

    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);
        state = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL, 8, 1);
        UEIENX |= 1<<RXSTPE;
    }
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
        if (!(UEINTX & 1<<RWAL))
        {
            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
            USBTask();

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
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)    // is setup received?
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

    while (!(UEINTX & 1<<TXINI))
    {
        uint8_t USB_DeviceState_LCL = state;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

#define FEATURE_SELFPOWERED_ENABLED     (1 << 0)
#define FEATURE_REMOTE_WAKEUP_ENABLED   (1 << 1)

ISR(USB_COM_vect, ISR_BLOCK)
{
    USBKB::instance->com();
}

ISR(USB_GEN_vect, ISR_BLOCK)
{
    USBKB::instance->gen();
}

USBKB *USBKB::instance;

void USBKB::Endpoint_ClearStatusStage()
{
    if (USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST)
    {
        while (!(UEINTX & 1<<RXOUTI))
            if (state == DEVICE_STATE_Unattached)
                return;

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }
    else
    {
        while (!(UEINTX & 1<<TXINI))
        {
            if (state == DEVICE_STATE_Unattached)
              return;
        }

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN
    }
}

void USBKB::USBTask()
{   
    uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();
    selectEndpoint(ENDPOINT_CONTROLEP);

    if (UEINTX & 1<<RXSTPI)     // is setup received?
        USB_Device_ProcessControlRequest();
    
    selectEndpoint(PrevEndpoint);
}


bool USBKB::Endpoint_ConfigureEndpoint_Prv(const uint8_t Number,
                                    const uint8_t UECFG0XData,
                                    const uint8_t UECFG1XData)
{
    for (uint8_t EPNum = Number; EPNum < ENDPOINT_TOTAL_ENDPOINTS; EPNum++)
    {
        uint8_t UECFG0XTemp;
        uint8_t UECFG1XTemp;
        uint8_t UEIENXTemp;

        selectEndpoint(EPNum);

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

        if (!(UESTA0X & 1<<CFGOK))
            return false;
    }

    selectEndpoint(Number);
    return true;
}

void USBKB::USB_Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t i = 0; i < sizeof(USB_Request_Header_t); i++)
        *(RequestHeader++) = read8();

    if (UEINTX & 1<<RXSTPI) // is setup received?
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
                {
                   uint8_t index = ((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);

                    if (index >= ENDPOINT_TOTAL_ENDPOINTS)
                        return;

                    selectEndpoint(index);
                    CurrentStatus = UECONX & 1<<STALLRQ;
                    selectEndpoint(ENDPOINT_CONTROLEP);
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
                uint8_t devAddr = USB_ControlRequest.wValue & 0x7F;
                UDADDR = (UDADDR & 1<<ADDEN) | (devAddr & 0x7f);
                UEINTX &= ~(1<<RXSTPI);
                Endpoint_ClearStatusStage();
                while (!(UEINTX & 1<<TXINI));
                UDADDR |= 1<<ADDEN;
                state = devAddr ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
            }
            break;
        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                const void* DescriptorPointer;
                uint16_t    DescriptorSize;

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
                    USB_Device_GetSerialString(sigDesc.UnicodeString);
                    UEINTX &= ~(1<<RXSTPI);
                    Endpoint_Write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((DescriptorSize = CALLBACK_USB_GetDescriptor(USB_ControlRequest.wValue,
                        USB_ControlRequest.wIndex, &DescriptorPointer)) == NO_DESCRIPTOR)
                {
                    return;
                }

                UEINTX &= ~(1<<RXSTPI);
                Endpoint_Write_Control_PStream_LE(DescriptorPointer, DescriptorSize);
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
                    if ((uint8_t)USB_ControlRequest.wValue > 1)
                        return;

                    UEINTX &= ~(1<<RXSTPI);
                    USB_Device_ConfigurationNumber = (uint8_t)USB_ControlRequest.wValue;
                    Endpoint_ClearStatusStage();

                    if (USB_Device_ConfigurationNumber)
                    {
                        state = DEVICE_STATE_Configured;
                    }
                    else
                    {
                        state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;
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



