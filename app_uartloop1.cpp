#include "misc.h"

static inline bool isUpper(char c) { return c >= 'A' && c <= 'Z'; }
static inline bool isLower(char c) { return c >= 'a' && c <= 'z'; }

static inline char convert(char c)
{
    if (isUpper(c)) return c + 32;
    if (isLower(c)) return c - 32;
    return c;
}

int main()
{
    Serial serial;
    serial.init();
    serial.enableRead();
    
    while (true)
        serial.write(convert(serial.readByte()));

    return 0;
}



