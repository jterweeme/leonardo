#ifndef _USBHID_H_
#define _USBHID_H_

static const uint8_t
    MAIN = 0,
    GLOBAL = 4,
    LOCAL = 8,
    USAGE = 0,
    USAGE_PAGE = 0,
    USAGE_MIN = 0x10,
    USAGE_MAX = 0x20,
    LOGICAL_MIN = 0x10,
    LOGICAL_MAX = 0x20,
    HID_INPUT = 0x80,
    HID_OUTPUT = 0x90,
    REPORT_SIZE = 0x70,
    REPORT_ID = 0x80,
    REPORT_COUNT = 0x90,
    COLLECTION = 0xa0,
    END_COLLECTION = 0xc0;


#endif



