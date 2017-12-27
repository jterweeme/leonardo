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
    KEYBOARD_EPSIZE = 8,
    HID_KEYBOARD_SC_ERROR_ROLLOVER = 0x01,
    HID_KEYBOARD_SC_POST_FAIL = 0x02,
    HID_KEYBOARD_SC_ERROR_UNDEFINED = 0x03,
    HID_KEYBOARD_SC_A = 0x04,
    HID_KEYBOARD_SC_B = 0x05,
    HID_KEYBOARD_SC_C = 0x06,
    HID_KEYBOARD_SC_D = 0x07,
    HID_KEYBOARD_SC_E = 0x08,
    HID_KEYBOARD_SC_F = 0x09,
    HID_KEYBOARD_SC_G = 0x0A,
    HID_KEYBOARD_SC_H = 0x0B,
    HID_KEYBOARD_SC_I = 0x0C,
    HID_KEYBOARD_SC_J = 0x0D,
    HID_KEYBOARD_SC_K = 0x0E,
    HID_KEYBOARD_SC_L = 0x0F,
    HID_KEYBOARD_SC_M = 0x10,
    HID_KEYBOARD_SC_N = 0x11,
    HID_KEYBOARD_SC_O = 0x12,
    HID_KEYBOARD_SC_P = 0x13,
    HID_KEYBOARD_SC_Q = 0x14,
    HID_KEYBOARD_SC_R = 0x15,
    HID_KEYBOARD_SC_S = 0x16,
    HID_KEYBOARD_SC_T = 0x17,
    HID_KEYBOARD_SC_U = 0x18,
    HID_KEYBOARD_SC_V = 0x19,
    HID_KEYBOARD_SC_W = 0x1A,
    HID_KEYBOARD_SC_X = 0x1B,
    HID_KEYBOARD_SC_Y = 0x1C,
    HID_KEYBOARD_SC_Z = 0x1D;

struct USB_KeyboardReport_Data_t
{
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keyCode[6];
}
__attribute__ ((packed));

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
    uint16_t getDescriptor(uint16_t value, uint8_t wIndex, const void **descAddr);
public:
    void sendReport(USB_KeyboardReport_Data_t &report);
    USBKB();
    void gen();
    void com();
    void usbTask();
};

#endif

