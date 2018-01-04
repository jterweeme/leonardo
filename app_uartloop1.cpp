#include "misc.h"

int main()
{
    Serial serial;
    serial.init();
    serial.enableRead();
    
    while (true)
        serial.write(Utility::convertCase(serial.readByte()));

    return 0;
}



