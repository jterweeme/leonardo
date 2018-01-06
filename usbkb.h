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
    HID_KEYBOARD_SC_Z = 0x1D,
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
    HID_KEYBOARD_SC_F1 = 0x3A,
    HID_KEYBOARD_SC_F2 = 0x3B,
    HID_KEYBOARD_SC_F3 = 0x3C,
    HID_KEYBOARD_SC_F4 = 0x3D,
    HID_KEYBOARD_SC_F5 = 0x3E,
    HID_KEYBOARD_SC_F6 = 0x3F,
    HID_KEYBOARD_SC_F7 = 0x40,
    HID_KEYBOARD_SC_F8 = 0x41,
    HID_KEYBOARD_SC_F9 = 0x42,
    HID_KEYBOARD_SC_F10 = 0x43,
    HID_KEYBOARD_SC_F11 = 0x44,
    HID_KEYBOARD_SC_F12 =  0x45,
    HID_KEYBOARD_SC_PRINT_SCREEN = 0x46,
    HID_KEYBOARD_SC_SCROLL_LOCK = 0x47,
    HID_KEYBOARD_SC_PAUSE = 0x48,
    HID_KEYBOARD_SC_INSERT = 0x49,
    HID_KEYBOARD_SC_HOME = 0x4A,
    HID_KEYBOARD_SC_PAGE_UP      = 0x4B,
    HID_KEYBOARD_SC_DELETE       = 0x4C,
    HID_KEYBOARD_SC_END          = 0x4D,
    HID_KEYBOARD_SC_PAGE_DOWN    = 0x4E,
    HID_KEYBOARD_SC_RIGHT_ARROW  = 0x4F,
    HID_KEYBOARD_SC_LEFT_ARROW   = 0x50,
    HID_KEYBOARD_SC_DOWN_ARROW   = 0x51,
    HID_KEYBOARD_SC_UP_ARROW     = 0x52,
    HID_KEYBOARD_MODIFIER_LEFTCTRL = 1<<0,
    HID_KEYBOARD_MODIFIER_LEFTSHIFT = 1<<1,
    HID_KEYBOARD_MODIFIER_LEFTALT    = 1<<2,
    HID_KEYBOARD_MODIFIER_LEFTGUI    = 1<<3,
    HID_KEYBOARD_MODIFIER_RIGHTCTRL  = 1<<4,
    HID_KEYBOARD_MODIFIER_RIGHTSHIFT = 1<<5,
    HID_KEYBOARD_MODIFIER_RIGHTALT   = 1<<6,
    HID_KEYBOARD_MODIFIER_RIGHTGUI   = 1<<7;

struct KBReport
{
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keyCode[6];
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
    void procCtrlReq();
    uint16_t getDescriptor(uint16_t value, uint8_t wIndex, const void **descAddr);
public:
    void sendReport(KBReport &report);
    USBKB();
};

#endif

