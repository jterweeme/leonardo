#include "busby.h"

const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
    {
        .Size = sizeof(USB_Descriptor_Device_t),
        .Type = DTYPE_Device
    },
    VERSION_BCD(01.10),
    USB_CSCP_NoDeviceClass,
    USB_CSCP_NoSpecificSubclass,
    USB_CSCP_NoSpecificProtocol,
    FIXED_CONTROL_ENDPOINT_SIZE,
    .VendorID               = 0x03EB,
    .ProductID              = 0x204B,
    .ReleaseNumber          = VERSION_BCD(00.01),
    .ManufacturerStrIndex   = 0x01,
    .ProductStrIndex        = 0x02,
    .SerialNumStrIndex      = USE_INTERNAL_SERIAL,
    .NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
    .Config =
    {
        {
            .Size = sizeof(USB_Descriptor_Configuration_Header_t),
            .Type = DTYPE_Configuration
        },

        sizeof(USB_Descriptor_Configuration_t),
        1,
        1,
        .ConfigurationStrIndex  = NO_DESCRIPTOR,
        .ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),
        .MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
    },

    .HID_Interface =
    {
        .Header =
        {
            .Size = sizeof(USB_Descriptor_Interface_t),
            .Type = DTYPE_Interface
        },
        .InterfaceNumber        = INTERFACE_ID_Mouse,
        .AlternateSetting       = 0,
        .TotalEndpoints         = 1,
        .Class                  = HID_CSCP_HIDClass,
        .SubClass               = HID_CSCP_BootSubclass,
        .Protocol               = HID_CSCP_MouseBootProtocol,
        .InterfaceStrIndex      = NO_DESCRIPTOR
    },

    .HID_MouseHID =
    {
        .Header =
        {
            .Size = sizeof(USB_HID_Descriptor_HID_t),
            .Type = HID_DTYPE_HID
        },

        .HIDSpec                = VERSION_BCD(1,1,1),
        .CountryCode            = 0x00,
        .TotalReportDescriptors = 1,
        .HIDReportType          = HID_DTYPE_Report,
        .HIDReportLength        = sizeof(MouseReport)
    },

    .HID_ReportINEndpoint =
    {
        .Header =
        {
            .Size = sizeof(USB_Descriptor_Endpoint_t),
            .Type = DTYPE_Endpoint
        },

        .EndpointAddress = MOUSE_EPADDR,
        .Attributes = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
        .EndpointSize = MOUSE_EPSIZE,
        .PollingIntervalMS = 0x05
    }


};

const USB_Descriptor_String_t PROGMEM LanguageString =
{
    .Header                 = {.Size = USB_STRING_LEN(1), .Type = DTYPE_String}
    //.UnicodeString          = (wchar_t)0x0409
};

const USB_Descriptor_String_t PROGMEM ManufacturerString =
{
    {
        .Size = USB_STRING_LEN(11), .Type = DTYPE_String
    },
    //L"Dean Camera"
};

const USB_Descriptor_String_t PROGMEM ProductString =
{
    {
        USB_STRING_LEN(22),
        DTYPE_String
    },
    L"LUFA USB-RS232 Adapter"
};

uint16_t USB::CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                     const uint8_t wIndex, const void** const DescriptorAddress)
{
    const uint8_t DescriptorType = wValue>>8;
    const uint8_t DescriptorNumber = (wValue & 0xFF);
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
                case 0x00:
                    Address = &LanguageString;
                    Size = pgm_read_byte(&LanguageString.Header.Size);
                    break;
                case 0x01:
                    Address = &ManufacturerString;
                    Size = pgm_read_byte(&ManufacturerString.Header.Size);
                    break;
                case 0x02:
                    Address = &ProductString;
                    Size = pgm_read_byte(&ProductString.Header.Size);
                    break;
            }

            break;
    }
   *DescriptorAddress = Address;
    return Size;
}



