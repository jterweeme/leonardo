#include "usbkb.h"
#include <avr/pgmspace.h>
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
    8,
    0x03EB,
    0x2042,
    0x0001,
    STRING_ID_Manufacturer,
    STRING_ID_Product,
    NO_DESCRIPTOR,
    1
};

struct USB_Descriptor_Configuration_t
{
    DescConf Config;
    DescIface HID_Interface;
    USB_HID_Descriptor_HID_t HID_KeyboardHID;
    DescEndpoint HID_ReportINEndpoint;
    DescEndpoint HID_ReportOUTEndpoint;
}
__attribute__ ((packed));

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
    0x00 | 0x80 | 0x01, (0<<0 | 1<<1 | 0<<2) & 0xFF,
    0x04 | 0x90 | 0x01, ((0x01) & 0xFF),
    0x04 | 0x70 | 0x01, ((0x08) & 0xFF),
    0x00 | 0x80 | 0x01, (((1 << 0)) & 0xFF),
    0x04 | 0x00 | 0x01, 0x08 & 0xFF,
    0x08 | 0x10 | 0x01, 0x01 & 0xFF,
    0x08 | 0x20 | 0x01, 0x05 & 0xFF,
    0x04 | 0x90 | 0x01, 0x05 & 0xFF,
    0x04 | 0x70 | 0x01, 0x01 & 0xFF, 
    0x00 | 0x90 | 0x01, ((0 << 0) | (1 << 1) | (0 << 2) | (0 << 7)) & 0xFF,
    0x04 | 0x90 | 0x01, ((0x01) & 0xFF),
    0x04 | 0x70 | 0x01, ((0x03) & 0xFF),
    0x00 | 0x90 | 0x01, (((1 << 0)) & 0xFF),
    0x04 | 0x10 | 0x01, 0x00 & 0xFF,
    0x04 | 0x20 | 0x01, 0x65 & 0xFF,
    0x04 | 0x00 | 0x01, 0x07 & 0xFF,
    0x08 | 0x10 | 0x01, 0x00 & 0xFF,
    0x08 | 0x20 | 0x01, 0x65 & 0xFF,
    0x04 | 0x90 | 0x01, 0x06 & 0xFF,
    0x04 | 0x70 | 0x01, 0x08 & 0xFF,
    0x00 | 0x80 | 0x01, (((0 << 0) | (0 << 1) | (0 << 2)) & 0xFF),
    0x00 | 0xC0 | 0x00
};

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
    UHWCON |= 1<<UVREGE;    // usb reg on
    USB_IsInitialized = true;
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

            state = DEVICE_STATE_Powered;
        }
        else
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
                PLLCSR = 0;

            state = DEVICE_STATE_Unattached;
        }
    }
    
    if (UDINT & 1<<EORSTI && UDIEN & 1<<SUSPE)
    {
        UDIEN &= ~(1<<SUSPE);   // disable int suspe
        UDIEN |= 1<<WAKEUPI;    // enable int wakeup, moet dit niet 1<<WAKEUPE zijn?
        USBCON |= 1<<FRZCLK;

        if (!(USB_Options & USB_OPT_MANUAL_PLL))
            PLLCSR = 0;

        state = DEVICE_STATE_Suspended;
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)
    {
        PLLCSR = USB_PLL_PSC;
        PLLCSR = USB_PLL_PSC | 1<<PLLE;   // PLL on
        while (!(PLLCSR & 1<<PLOCK));   // PLL is ready?
        USBCON &= ~(1<<FRZCLK);
        UDINT &= ~(1<<WAKEUPI);
        UDIEN &= ~(1<<WAKEUPI);
        UDIEN |= 1<<SUSPE;

        if (USB_Device_ConfigurationNumber)
            state = DEVICE_STATE_Configured;
        else
            state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);      // clear INT EORSTI
        state = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);       // clear INT SUSPI
        UDIEN &= ~(1<<SUSPE);       // disable INT SUSPE
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

void USBKB::sendReport(USB_KeyboardReport_Data_t &report)
{
    _inpoint.select();

    writeStream(&report, sizeof(report), NULL);
    UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
    _outpoint.select();

    if (UEINTX & 1<<RXOUTI)
    {
        read8();
        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }
}

void USBKB::Device_ProcessControlRequest()
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
                    selectEndpoint((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);
                    CurrentStatus = UECONX & 1<<STALLRQ;
                    selectEndpoint(ENDPOINT_CONTROLEP);
                    break;
                default:
                    return;
                }

                UEINTX &= ~(1<<RXSTPI);
                write16(CurrentStatus);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
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
                state = DeviceAddress ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
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
                    write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((descSize = getDescriptor(USB_ControlRequest.wValue,
                          USB_ControlRequest.wIndex, &DescriptorPointer)) == NO_DESCRIPTOR)
                {
                    return;
                }

                UEINTX &= ~(1<<RXSTPI);     // clear setup
                write_Control_PStream_LE(DescriptorPointer, descSize);
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
            }

            break;
        case REQ_GetConfiguration:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                write8(USB_Device_ConfigurationNumber);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
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
                    state = DEVICE_STATE_Configured;
                else
                    state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

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


