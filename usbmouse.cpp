#include "usbmouse.h"
#include "usbhid.h"
#include <avr/pgmspace.h>

static constexpr uint8_t
    MOUSE_EPADDR = ENDPOINT_DIR_IN | 1,
    MOUSE_EPSIZE = 8,
    INTERFACE_ID_Mouse = 0,
    STRING_ID_Manufacturer = 1,
    STRING_ID_Product = 2;

const DescDev PROGMEM DeviceDescriptor =
{
    sizeof(DescDev),
    DTYPE_Device,
    0x0110,
    0,
    0,
    0,
    FIXED_CONTROL_ENDPOINT_SIZE,
    0x03EB,
    0x204B,
    0x0001,
    STRING_ID_Manufacturer,
    STRING_ID_Product,
    USE_INTERNAL_SERIAL,
    FIXED_NUM_CONFIGURATIONS
};

const uint8_t PROGMEM mouseReport[] =
{
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
};

struct MyConf
{
    DescConf config;
    DescIface hidInterface;
    HIDDesc HID_KeyboardHID;
    DescEndpoint HID_ReportINEndpoint;
};

const MyConf PROGMEM myConf =
{
    {
        sizeof(MyConf),
        DTYPE_Configuration,
        sizeof(MyConf),
        1,
        1,
        0,
        USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED,
        USB_CONFIG_POWER_MA(100)
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        INTERFACE_ID_Mouse,
        0,
        1,
        HID_CSCP_HIDClass,
        HID_CSCP_BootSubclass,
        HID_CSCP_MouseBootProtocol,
        0
    },
    {
        {
            sizeof(HIDDesc),
            HID_DTYPE_HID
        },

        0x0111,
        0x00,
        1,
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
    DTYPE_String,
    (wchar_t)0x0409
};

const DescString<12> PROGMEM manufacturerString =
{
    USB_STRING_LEN(11),
    DTYPE_String,
    L"Dean Camera"
};

const DescString<10> PROGMEM productString =
{
    USB_STRING_LEN(9),
    DTYPE_String,
    L"USB Mouse"
};

uint16_t USBMouse::getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddr)
{
    const uint8_t DescriptorType = wValue>>8;
    const uint8_t DescriptorNumber = (wValue & 0xFF);
    const void* Address = NULL;
    uint16_t    Size    = NO_DESCRIPTOR;

    switch (DescriptorType)
    {
        case DTYPE_Device:
            Address = &DeviceDescriptor;
            Size = sizeof(DescDev);
            break;
        case DTYPE_Configuration:
            //Address = &ConfigurationDescriptor;
            Size = sizeof(MyConf);
            break;
        case DTYPE_String:
            switch (DescriptorNumber)
            {
                case 0x00:
                    Address = &languageString;
                    Size = pgm_read_byte(&languageString.size);
                    break;
                case 0x01:
                    Address = &manufacturerString;
                    Size = pgm_read_byte(&manufacturerString.size);
                    break;
                case 0x02:
                    Address = &productString;
                    Size = pgm_read_byte(&productString.size);
                    break;
            }

            break;
    }
    *descAddr = Address;
    return Size;
}

USBMouse::USBMouse() : _inpoint(0, 0, 0, 1)
{
}


