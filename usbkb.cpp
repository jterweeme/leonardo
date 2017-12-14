#include "usbkb.h"
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "misc.h"

static const uint8_t
    HID_REQ_GetReport = 0x01,
    HID_REQ_GetIdle = 0x02,
    HID_REQ_GetProtocol = 0x03,
    HID_REQ_SetReport = 0x09,
    HID_REQ_SetIdle = 0x0a,
    HID_REQ_SetProtocol = 0x0b,
    STRING_ID_Manufacturer = 1,
    STRING_ID_Product = 2;

const DescDev PROGMEM DeviceDescriptor =
{
    {
        sizeof(DescDev),
        DTYPE_Device
    },
    0x0110,
    USB_CSCP_NoDeviceClass,
    USB_CSCP_NoSpecificSubclass,
    USB_CSCP_NoSpecificProtocol,
    FIXED_CONTROL_ENDPOINT_SIZE,
    0x03EB,
    0x204B,
    0x0001,
    STRING_ID_Manufacturer,
    STRING_ID_Product,
    NO_DESCRIPTOR,
    FIXED_NUM_CONFIGURATIONS
};

struct USB_KeyboardReport_Data_t
{
    uint8_t Modifier;
    uint8_t Reserved;
    uint8_t KeyCode[6];
}
__attribute__ ((packed));

USB_KeyboardReport_Data_t *KeyboardReport;

const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
    {
        {
            sizeof(DescConf),
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
            sizeof(DescIface),
            DTYPE_Interface
        },
        INTERFACE_ID_Keyboard,
        0,
        1,
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

        .HIDSpec = 0x0111,
        .CountryCode            = 0x00,
        .TotalReportDescriptors = 1,
        .HIDReportType          = HID_DTYPE_Report,
        .HIDReportLength        = sizeof(KeyboardReport)
    },
    {   // HID_ReportINEndpoint
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },

        KEYBOARD_EPADDR,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        KEYBOARD_EPSIZE,
        0x05
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

USBKB::USBKB() :
    _inpoint(KEYBOARD_IN_EPADDR, KEYBOARD_EPSIZE, EP_TYPE_INTERRUPT, 1),
    _outpoint(KEYBOARD_OUT_EPADDR, KEYBOARD_EPSIZE, EP_TYPE_INTERRUPT, 1)
{
    clock_prescale_set(clock_div_2);

    if (!(USB_Options & USB_OPT_REG_DISABLED))
        UHWCON |= 1<<UVREGE;    // usb reg on
    else
        UHWCON &= ~(1<<UVREGE); // usb reg off

    USB_IsInitialized = true;
    USBCON &= ~(1<<VBUSTE);
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1<<USBE);   // disable usb controller
    USBCON |= 1<<USBE;      // enable usb controller
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

    USBCON |= 1<<VBUSTE;        // enable vbus int
    configureEndpoint(_control.addr, _control.type, _control.size, 1);
    UDINT &= ~(1<<SUSPI);
    UDIEN |= 1<<SUSPE;
    UDIEN |= 1<<EORSTE;
    UDCON &= ~(1<<DETACH);
    USBCON |= 1<<OTGPADE;
    sei();
}

ISR(USB_GEN_vect)
{
    USBKB::instance->gen();
}

ISR(USB_COM_vect)
{
    USBKB::instance->com();
}

void USBKB::usbTask()
{
#if 0
    if (USB_DeviceState == DEVICE_STATE_Unatached)
        return;
#endif
    uint8_t prevEndpoint = getCurrentEndpoint();
    _control.select();

    if (UEINTX & 1<<RXSTPI)
        Device_ProcessControlRequest();

    selectEndpoint(prevEndpoint);
}

void USBKB::gen()
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE)
        UDINT &= ~(1<<SOFI);

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
            {
                PLLCSR = USB_PLL_PSC;
                PLLCSR = USB_PLL_PSC | 1<<PLLE;
                while (!(PLLCSR & 1<<PLOCK));
            }

            GPIOR0 = DEVICE_STATE_Powered;
        }
        else
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
                PLLCSR = 0;

            GPIOR0 = DEVICE_STATE_Unattached;
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
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);
        GPIOR0 = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        configureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL, _control.size, 1);
        UEIENX |= 1<<RXSTPE;
    }

}

void USBKB::com()
{
    uint8_t prevSelectedEndp = getCurrentEndpoint();
    _control.select();
    UEIENX &= ~(1<<RXSTPE);
    sei();
    Device_ProcessControlRequest();
    _control.select();
    UEIENX |= 1<<RXSTPE;
    selectEndpoint(prevSelectedEndp);
}

void USBKB::EVENT_USB_Device_ControlRequest()
{
    if (!(UEINTX & 1<<RXSTPI))      // is setup received?
        return;

    if (USB_ControlRequest.wIndex != _control.addr)
        return;

    switch (USB_ControlRequest.bRequest)
    {
        case HID_REQ_GetReport:

            break;
    }
}

uint16_t USBKB::getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddress)
{
    const uint8_t descType = wValue >> 8;
    const uint8_t descNumber = wValue & 0xFF;
    const void* Address = NULL;
    uint16_t    Size    = NO_DESCRIPTOR;

    switch (descType)
    {
        case DTYPE_Device:
            Address = &DeviceDescriptor;
            Size = sizeof(DescDev);
            break;
        case DTYPE_Configuration:
            Address = &ConfigurationDescriptor;
            Size = sizeof(USB_Descriptor_Configuration_t);
            break;
        case DTYPE_String:
            switch (descNumber)
            {
                case 0x00:
                    Address = &LanguageString;
                    Size = pgm_read_byte(&LanguageString.Header.size);
                    break;
                case 0x01:
                    Address = &ManufacturerString;
                    Size = pgm_read_byte(&ManufacturerString.Header.size);
                    break;
                case 0x02:
                    Address = &ProductString;
                    Size = pgm_read_byte(&ProductString.Header.size);
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

    *descAddress = Address;
    return Size;
}


void USBKB::Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t i = 0; i < sizeof(USB_Request_Header_t); i++)
    {
        *(RequestHeader++) = read8();
    }

#if 0
    char buf[50];
    snprintf(buf, 50, "%u\r\n", USB_ControlRequest.bmRequestType);
    Serial::instance->write(buf);
#endif
    EVENT_USB_Device_ControlRequest();

    if (UEINTX & 1<<RXSTPI)
    {
        uint8_t bmRequestType = USB_ControlRequest.bRequest;

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
                        selectEndpoint((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);
                        CurrentStatus = UECONX & 1<<STALLRQ;
                        selectEndpoint(ENDPOINT_CONTROLEP);
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

                if (USB_ControlRequest.wValue == (DTYPE_String << 8 | USE_INTERNAL_SERIAL))
                {
                    struct
                    {
                        DescHeader Header;
                        uint16_t UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
                    }
                    sigDesc;

                    sigDesc.Header.type = DTYPE_String;
                    sigDesc.Header.size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
                    Device_GetSerialString(sigDesc.UnicodeString);
                    UEINTX &= ~(1<<RXSTPI);
                    Endpoint_Write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((descSize = getDescriptor(USB_ControlRequest.wValue,
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
                write8(USB_Device_ConfigurationNumber);
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

                configureEndpoint(_inpoint.addr, _inpoint.type, _inpoint.size, _inpoint.banks);
                configureEndpoint(_outpoint.addr, _outpoint.type, _outpoint.size, _outpoint.banks);
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


