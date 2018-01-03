#include "usbkb.h"
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "misc.h"
#include "usbhid.h"

static constexpr uint8_t
    HID_REQ_GetReport = 0x01,
    HID_REQ_GetIdle = 0x02,
    HID_REQ_GetProtocol = 0x03,
    HID_REQ_SetReport = 0x09,
    HID_REQ_SetIdle = 0x0a,
    HID_REQ_SetProtocol = 0x0b,
    STRING_ID_Language = 0,
    STRING_ID_Manufacturer = 1,
    STRING_ID_Product = 2;

const DescDev PROGMEM devDesc =
{
    {
        sizeof(DescDev),
        DTYPE_Device
    },
    0x0110,
    USB_CSCP_NoDeviceClass,
    USB_CSCP_NoSpecificSubclass,
    USB_CSCP_NoSpecificProtocol,
    8,
    0x03EB,
    0x2042,
    0x0001,
    STRING_ID_Manufacturer,
    STRING_ID_Product,
    0,
    1
};

struct MyConf
{
    DescConf config;
    DescIface HID_Interface;
    USB_HID_Descriptor_HID_t HID_KeyboardHID;
    DescEndpoint HID_ReportINEndpoint;
    DescEndpoint HID_ReportOUTEndpoint;
}
__attribute__ ((packed));

const uint8_t PROGMEM KeyboardReport[] =
{
    GLOBAL | USAGE_PAGE     | 1, 0x01,
    LOCAL  | USAGE          | 1, 0x06,
    MAIN   | COLLECTION     | 1, 0x01,
    GLOBAL | USAGE_PAGE     | 1, 0x07,
    LOCAL  | USAGE_MIN      | 1, 0xE0,
    LOCAL  | USAGE_MAX      | 1, 0xE7,
    GLOBAL | LOGICAL_MIN    | 1, 0x00,
    GLOBAL | LOGICAL_MAX    | 1, 0x01,
    GLOBAL | REPORT_SIZE    | 1, 0x01,
    GLOBAL | REPORT_COUNT   | 1, 0x08,
    MAIN   | HID_INPUT      | 1, 1<<1,
    GLOBAL | REPORT_COUNT   | 1, 0x01,
    GLOBAL | REPORT_SIZE    | 1, 0x08,
    MAIN   | HID_INPUT      | 1, 1<<0,
    GLOBAL | USAGE_PAGE     | 1, 0x08,
    LOCAL  | USAGE_MIN      | 1, 0x01,
    LOCAL  | USAGE_MAX      | 1, 0x05,
    GLOBAL | REPORT_COUNT   | 1, 0x05,
    GLOBAL | REPORT_SIZE    | 1, 0x01, 
    MAIN   | OUTPUT         | 1, 1<<1,
    GLOBAL | REPORT_COUNT   | 1, 0x01,
    GLOBAL | REPORT_SIZE    | 1, 0x03,
    MAIN   | OUTPUT         | 1, 1<<0,
    GLOBAL | LOGICAL_MIN    | 1, 0x00,
    GLOBAL | LOGICAL_MAX    | 1, 0x65,
    GLOBAL | USAGE_PAGE     | 1, 0x07,
    LOCAL  | USAGE_MIN      | 1, 0x00,
    LOCAL  | USAGE_MAX      | 1, 0x65,
    GLOBAL | REPORT_COUNT   | 1, 0x06,
    GLOBAL | REPORT_SIZE    | 1, 0x08,
    MAIN   | HID_INPUT      | 1, 0,
    MAIN   | END_COLLECTION | 0
};

const MyConf PROGMEM myConf =
{
    {
        {
            sizeof(DescConf),
            DTYPE_Configuration
        },

        sizeof(MyConf),
        1,
        1,
        0,
        USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED,
        USB_CONFIG_POWER_MA(100)
    },
    {   // HID_Interface
        {
            sizeof(DescIface),
            DTYPE_Interface
        },
        INTERFACE_ID_Keyboard,
        0,
        2,
        HID_CSCP_HIDClass,
        HID_CSCP_BootSubclass,
        HID_CSCP_KeyboardBootProtocol,
        0
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
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },

        ENDPOINT_DIR_IN | 1,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        8,
        5
    },
    {   // HID_ReportOUTEndpoint
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },

        ENDPOINT_DIR_OUT | 2,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        8,
        5
    }
};

const USB_Descriptor_String_t<2> PROGMEM languageString =
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

const USB_Descriptor_String_t<13> PROGMEM ProductString =
{
    {
        USB_STRING_LEN(12),
        DTYPE_String
    },
    L"USB Keyboard"
};

USBKB::USBKB() :
    _inpoint(KEYBOARD_IN_EPADDR, KEYBOARD_EPSIZE, EP_TYPE_INTERRUPT, 1),
    _outpoint(KEYBOARD_OUT_EPADDR, KEYBOARD_EPSIZE, EP_TYPE_INTERRUPT, 1)
{
    UHWCON |= 1<<UVREGE;    // usb reg on
    USBCON &= ~(1<<VBUSTE); // disable INT vbuste
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1<<USBE);   // disable usb controller
    USBCON |= 1<<USBE;      // enable usb controller
    USBCON &= ~(1<<FRZCLK);
    PLLCSR = 0;
    state = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    UDCON &= ~(1<<LSM);         // full speed
    USBCON |= 1<<VBUSTE;        // enable vbus int
    configureEndpoint(_control.addr, _control.type, _control.size, 1);
    UDINT &= ~(1<<SUSPI);
    UDIEN |= 1<<SUSPE;
    UDIEN |= 1<<EORSTE;
    UDCON &= ~(1<<DETACH);
    USBCON |= 1<<OTGPADE;
    sei();
}

uint16_t USBKB::getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddr)
{
    const uint8_t descNumber = wValue & 0xFF;
    const void *addr = NULL;
    uint16_t size = 0;

    switch (wValue >> 8)
    {
    case DTYPE_Device:
        addr = &devDesc;
        size = sizeof(DescDev);
        break;
    case DTYPE_Configuration:
        addr = &myConf;
        size = sizeof(MyConf);
        break;
    case DTYPE_String:
        switch (descNumber)
        {
        case STRING_ID_Language:
            addr = &languageString;
            size = pgm_read_byte(&languageString.header.size);
            break;
        case STRING_ID_Manufacturer:
            addr = &ManufacturerString;
            size = pgm_read_byte(&ManufacturerString.header.size);
            break;
        case STRING_ID_Product:
            addr = &ProductString;
            size = pgm_read_byte(&ProductString.header.size);
            break;
        }

        break;
    case HID_DTYPE_HID:
        addr = &myConf.HID_KeyboardHID;
        size = sizeof(USB_HID_Descriptor_HID_t);
        break;
    case HID_DTYPE_Report:
        addr = &KeyboardReport;
        size = sizeof(KeyboardReport);
        break;
    }

    *descAddr = addr;
    return size;
}

void USBKB::sendReport(KBReport &report)
{
    _inpoint.select();
    writeStream2(&report, sizeof(report), NULL);
    UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
    _outpoint.select();

    if (UEINTX & 1<<RXOUTI)
    {
        if (UEINTX & 1<<RWAL)
            read8();

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }
}

void USBKB::Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&_ctrlReq;

    for (uint8_t i = 0; i < sizeof(USBRequest); i++)
        *(RequestHeader++) = read8();

    if (UEINTX & 1<<RXSTPI) // is setup received?
    {
        uint8_t bmRequestType = _ctrlReq.bmRequestType;

        switch (_ctrlReq.bRequest)
        {
        case REQ_GetStatus:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                uint8_t CurrentStatus = 0;

                switch (_ctrlReq.bmRequestType)
                {
                case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE):
                    if (USB_Device_CurrentlySelfPowered)
                        CurrentStatus |= FEATURE_SELFPOWERED_ENABLED;

                    if (USB_Device_RemoteWakeupEnabled)
                        CurrentStatus |= FEATURE_REMOTE_WAKEUP_ENABLED;

                    break;
                case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT):
                    selectEndpoint((uint8_t)_ctrlReq.wIndex & ENDPOINT_EPNUM_MASK);
                    CurrentStatus = UECONX & 1<<STALLRQ;
                    selectEndpoint(ENDPOINT_CONTROLEP);
                    break;
                default:
                    return;
                }

                UEINTX &= ~(1<<RXSTPI);
                write16(CurrentStatus);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
                clearStatusStage();
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
                uint8_t devAddr = _ctrlReq.wValue & 0x7F;
                UDADDR = UDADDR & 1<<ADDEN | devAddr & 0x7F;
                UEINTX &= ~(1<<RXSTPI);
                clearStatusStage();
                while ((UEINTX & 1<<TXINI) == 0);
                UDADDR |= 1<<ADDEN; // enable dev addr
                state = devAddr ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
            }
            break;
        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                const void *descPtr;
                uint16_t descSize;

                if (_ctrlReq.wValue == (DTYPE_String << 8 | USE_INTERNAL_SERIAL))
                {
                    SigDesc sigDesc;
                    sigDesc.header.type = DTYPE_String;
                    sigDesc.header.size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
                    Device_GetSerialString(sigDesc.unicodeString);
                    UEINTX &= ~(1<<RXSTPI);
                    write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((descSize = getDescriptor(_ctrlReq.wValue, _ctrlReq.wIndex, &descPtr)) == 0)
                    return;

                UEINTX &= ~(1<<RXSTPI);     // clear setup
                write_Control_PStream_LE(descPtr, descSize);
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
            }

            break;
        case REQ_GetConfiguration:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                write8(USB_Device_ConfigurationNumber);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
                clearStatusStage();
            }

            break;

        case REQ_SetConfiguration:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                if ((uint8_t)_ctrlReq.wValue > FIXED_NUM_CONFIGURATIONS)
                    return;

                UEINTX &= ~(1<<RXSTPI);
                USB_Device_ConfigurationNumber = (uint8_t)_ctrlReq.wValue;
                clearStatusStage();

                if (USB_Device_ConfigurationNumber)
                    state = DEVICE_STATE_Configured;
                else
                    state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

                configureEndpoint(_inpoint);
                configureEndpoint(_outpoint);
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


