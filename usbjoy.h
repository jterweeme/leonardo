#ifndef _USBJOY_H_
#define _USBJOY_H_
#include <stdint.h>
#include "usbhid.h"

const uint8_t PROGMEM JoystickReport[] =
{
    GLOBAL | USAGE_PAGE | 1, 0x01,
    LOCAL  | USAGE      | 1, 0x04,
    MAIN   | COLLECTION | 1, 0x01,
    GLOBAL | USAGE_PAGE | 1, 0x09,
    LOCAL  | USAGE_MIN  | 1, 0x01,
    LOCAL  | USAGE_MAX  | 1, 0x06,
    GLOBAL | LOGICAL_MIN | 1, 0x00,
    GLOBAL | LOGICAL_MAX | 1, 0x01,
    GLOBAL | REPORT_COUNT | 1, 0x06,
    GLOBAL | REPORT_SIZE | 1, 0x01,
    
};

#endif



