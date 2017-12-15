#ifndef _USBKB_H_
#define _USBKB_H_

#include "busby.h"

static constexpr uint8_t
    HID_DTYPE_HID = 0x21,
    HID_DTYPE_Report = 0x22,
    MOUSE_EPADDR = ENDPOINT_DIR_IN | 1,
    MOUSE_EPSIZE = 8,
    HID_CSCP_BootSubclass = 1,
    HID_CSCP_KeyboardBootProtocol = 1,
    HID_CSCP_MouseBootProtocol = 2,
    HID_CSCP_HIDClass = 3,
    INTERFACE_ID_Mouse = 0,
    INTERFACE_ID_Keyboard = 0,
    KEYBOARD_EPADDR = ENDPOINT_DIR_IN | 1,
    KEYBOARD_IN_EPADDR = ENDPOINT_DIR_IN | 1,
    KEYBOARD_OUT_EPADDR = ENDPOINT_DIR_OUT | 2,
    KEYBOARD_EPSIZE = 8;

struct USB_CDC_Descriptor_FunctionalUnion_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint8_t MasterInterfaceNumber;
    uint8_t SlaveInterfaceNumber;
}
__attribute__ ((packed));

struct USB_CDC_Descriptor_FunctionalACM_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint8_t Capabilities;
}
__attribute__ ((packed));


struct USB_CDC_Descriptor_FunctionalHeader_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint16_t CDCSpecification;
}
__attribute__ ((packed));

struct USB_HID_Descriptor_HID_t
{
    DescHeader Header; /**< Regular descriptor header cd length. */
    uint16_t HIDSpec; /**< BCD encoded vernd device complies to. */
    uint8_t CountryCode; /**< Country codce, or zero if universal. */
    uint8_t TotalReportDescriptors; /**< riptors for the interface. */
    uint8_t HIDReportType; /**< Tyort, set to \ref HID_DTYPE_Report. */
    uint16_t HIDReportLength; /**< report descriptor, in bytes. */
}
__attribute__ ((packed));

class USBKB : public USB
{
private:
    Endpoint _inpoint;
    Endpoint _outpoint;
    void Device_ProcessControlRequest();
    void EVENT_USB_Device_ControlRequest();
    uint16_t getDescriptor(uint16_t value, uint8_t wIndex, const void **descAddr);
public:
    USBKB();
    void gen();
    void com();
    void usbTask();
};

#endif

