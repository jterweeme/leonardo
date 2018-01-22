#include "usbmouse.h"
#include "usbhid.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

static constexpr uint8_t
    MOUSE_EPADDR = ENDPOINT_DIR_IN | 1,
    MOUSE_EPSIZE = 8,
    INTERFACE_ID_Mouse = 0,
    STRING_ID_LANGUAGE = 0,
    STRING_ID_MANUFACTURER = 1,
    STRING_ID_PRODUCT = 2;

const DescDev PROGMEM DeviceDescriptor =
{
    sizeof(DescDev),
    DTYPE_DEVICE,
    0x0110,
    0,
    0,
    0,
    FIXED_CONTROL_ENDPOINT_SIZE,
    0x03EB,
    0x2041,
    0x0001,
    STRING_ID_MANUFACTURER,
    STRING_ID_PRODUCT,
    0,
    FIXED_NUM_CONFIGURATIONS
};

const uint8_t PROGMEM mouseReport[] =
{
#if 0
    GLOBAL | USAGE_PAGE     | 1, 0x01,
    LOCAL  | USAGE          | 1, 0x02,  // mouse
    MAIN   | COLLECTION     | 1, 0x01,
    GLOBAL | USAGE_PAGE     | 1, 0x09,  // pointer
    LOCAL  | USAGE_MIN      | 1, 0x01,
    LOCAL  | USAGE_MAX      | 1, 0x03,
    GLOBAL | LOGICAL_MIN    | 1, 0x00,
    GLOBAL | LOGICAL_MAX    | 1, 0x01,
    GLOBAL | REPORT_COUNT   | 1, 0x03,
    GLOBAL | REPORT_SIZE    | 1, 0x01,
    MAIN   | HID_INPUT      | 1, 1<<1,
    GLOBAL | REPORT_COUNT   | 1, 0x01,
    GLOBAL | REPORT_SIZE    | 1, 0x05,
    MAIN   | HID_INPUT      | 1, 1<<0,
    GLOBAL | USAGE_PAGE     | 1, 0x01,
    LOCAL  | USAGE          | 1, 0x30,
    LOCAL  | USAGE          | 1, 0x31,
    GLOBAL | LOGICAL_MIN    | 1, 0,
    GLOBAL | LOGICAL_MAX    | 1, 1,
    GLOBAL | PHYSICAL_MIN   | 1, 0,
    GLOBAL | PHYSICAL_MAX   | 1, 1,
    GLOBAL | REPORT_COUNT   | 1, 0x02,
    GLOBAL | REPORT_SIZE    | 1, 0x08,
    MAIN   | HID_INPUT      | 1, 0,
    MAIN   | END_COLLECTION | 0
#else
    (0x04 | 0x00 | 0x01), ((0x01) & 0xFF),
    (0x08 | 0x00 | 0x01), ((0x02) & 0xFF),
    (0x00 | 0xA0 | 0x01), ((0x01) & 0xFF),
    (0x08 | 0x00 | 0x01), ((0x01) & 0xFF),
    (0x00 | 0xA0 | 0x01), ((0x00) & 0xFF),
    (0x04 | 0x00 | 0x01), ((0x09) & 0xFF),
    (0x08 | 0x10 | 0x01), ((0x01) & 0xFF),
    (0x08 | 0x20 | 0x01), ((0x03) & 0xFF),
    (0x04 | 0x10 | 0x01), ((0x00) & 0xFF),
    (0x04 | 0x20 | 0x01), ((0x01) & 0xFF),
    (0x04 | 0x90 | 0x01), ((0x03) & 0xFF),
    (0x04 | 0x70 | 0x01), ((0x01) & 0xFF),
    (0x00 | 0x80 | 0x01), (((0 << 0) | (1 << 1) | (0 << 2)) & 0xFF),
    (0x04 | 0x90 | 0x01), ((0x01) & 0xFF),
    (0x04 | 0x70 | 0x01), ((0x05) & 0xFF),
    (0x00 | 0x80 | 0x01), (((1 << 0)) & 0xFF),
    (0x04 | 0x00 | 0x01), ((0x01) & 0xFF),
    (0x08 | 0x00 | 0x01), ((0x30) & 0xFF),
    (0x08 | 0x00 | 0x01), ((0x31) & 0xFF),
    (0x04 | 0x10 | 0x01), ((-1) & 0xFF),
    (0x04 | 0x20 | 0x01), ((1) & 0xFF),
    (0x04 | 0x30 | 0x01), ((-1) & 0xFF),
    (0x04 | 0x40 | 0x01), ((1) & 0xFF),
    (0x04 | 0x90 | 0x01), ((0x02) & 0xFF),
    (0x04 | 0x70 | 0x01), ((0x08) & 0xFF),
    (0x00 | 0x80 | 0x01), (((0<<0) | (1<<1) | (1<<2)) & 0xFF),
    (0x00 | 0xC0 | 0x00),
    (0x00 | 0xC0 | 0x00)
#endif
};

struct MyConf
{
    DescConf config;
    DescIface hidInterface;
    HIDDesc mouseHID;
    DescEndpoint inpoint;
};

const MyConf PROGMEM myConf =
{
    {
        sizeof(MyConf),
        DTYPE_CONFIGURATION,
        sizeof(MyConf),
        1,      // 1 interface
        1,      // configuration number = 1
        0,
        USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED,
        USB_CONFIG_POWER_MA(100)
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        INTERFACE_ID_Mouse,
        0,      // alternate settings
        1,      // 1 endpoint
        HID_CSCP_HIDClass,
        HID_CSCP_BootSubclass,
        HID_CSCP_MouseBootProtocol,
        0
    },
    {
        sizeof(HIDDesc),
        HID_DTYPE_HID,
        0x0111,
        0x00,
        1,      // 1 report descriptor
        HID_DTYPE_Report,
        sizeof(mouseReport)
    },
    {
        sizeof(DescEndpoint),
        DTYPE_Endpoint,
        MOUSE_EPADDR,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        MOUSE_EPSIZE,
        0x05
    }
};

const DescString<2> PROGMEM languageString =
{
    USB_STRING_LEN(1),
    DTYPE_STRING,
    (wchar_t)0x0409
};

const DescString<12> PROGMEM manufacturerString =
{
    USB_STRING_LEN(11),
    DTYPE_STRING,
    L"Dean Camera"
};

const DescString<10> PROGMEM productString =
{
    USB_STRING_LEN(9),
    DTYPE_STRING,
    L"USB Mouse"
};

uint16_t USBMouse::getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddr)
{
    const void *addr = NULL;
    uint16_t size = 0;

    switch (wValue >> 8)
    {
    case DTYPE_DEVICE:
        addr = &DeviceDescriptor;
        size = sizeof(DescDev);
        break;
    case DTYPE_CONFIGURATION:
        addr = &myConf;
        size = sizeof(MyConf);
        break;
    case DTYPE_STRING:
        switch (wValue & 0xff)
        {
        case STRING_ID_LANGUAGE:
            addr = &languageString;
            size = pgm_read_byte(&languageString.size);
            break;
        case STRING_ID_MANUFACTURER:
            addr = &manufacturerString;
            size = pgm_read_byte(&manufacturerString.size);
            break;
        case STRING_ID_PRODUCT:
            addr = &productString;
            size = pgm_read_byte(&productString.size);
            break;
        }

        break;
    case HID_DTYPE_HID:
        addr = &myConf.mouseHID;
        size = sizeof(HIDDesc);
        break;
    case HID_DTYPE_Report:
        addr = &mouseReport;
        size = sizeof(mouseReport);
        break;
    }

    *descAddr = addr;
    return size;
}

USBMouse::USBMouse() : _inpoint(ENDPOINT_DIR_IN | 1, 8, EP_TYPE_INTERRUPT, 1)
{
    *p_uhwcon |= 1<<UVREGE;    // usb reg on
    *p_usbcon &= ~(1<<vbuste); // disable INT vbuste
    *p_udien = 0;
    *p_usbint = 0;
    *p_udint = 0;
    *p_usbcon &= ~(1<<USBE);   // disable usb controller
    *p_usbcon |= 1<<USBE;      // enable usb controller
    *p_usbcon &= ~(1<<FRZCLK);
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

void USBMouse::procCtrlReq()
{
    uint8_t* RequestHeader = (uint8_t*)&_ctrlReq;

    for (uint8_t i = 0; i < sizeof(USBRequest); i++)
        *(RequestHeader++) = read8();

    if (*p_ueintx & 1<<rxstpi) // is setup received?
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
                UDADDR = (UDADDR & 1<<ADDEN) | (devAddr & 0x7F);
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

                if (_ctrlReq.wValue == (DTYPE_STRING << 8 | USE_INTERNAL_SERIAL))
                {
                    SigDesc sigDesc;
                    sigDesc.type = DTYPE_STRING;
                    sigDesc.size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
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
                *p_ueintx &= ~(1<<rxstpi); // clear setup
                write8(USB_Device_ConfigurationNumber);
                *p_ueintx &= ~(1<<txini | 1<<fifocon); // clear in
                clearStatusStage();
            }

            break;
        case REQ_SetConfiguration:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                if ((uint8_t)_ctrlReq.wValue > FIXED_NUM_CONFIGURATIONS)
                    return;

                *p_ueintx &= ~(1<<rxstpi);
                USB_Device_ConfigurationNumber = (uint8_t)_ctrlReq.wValue;
                clearStatusStage();

                if (USB_Device_ConfigurationNumber)
                    state = DEVICE_STATE_Configured;
                else
                    state = *p_udaddr & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

                configureEndpoint(_inpoint);
                *p_udien |= 1<<sofe;
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

void USBMouse::sendReport(MouseReportData &mouseReport)
{
    if (*p_ueintx & 1<<rwal)
    {
        writeStream2(&mouseReport, sizeof(mouseReport), NULL);
        *p_ueintx &= ~(1<<txini | 1<<fifocon);
    }
}



