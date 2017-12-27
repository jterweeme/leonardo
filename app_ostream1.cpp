#include "stream.h"

inline char nibble(uint8_t n)
{
    return n <= 9 ? '0' + n : 'A' + n - 10;
}

int main()
{
    CDC cdc;
    USBStream os(&cdc);
    
    while (true)
    {
        for (uint32_t i = 0; i < 0xffffffff; i++)
        {
            for (int8_t j = 7; j >= 0; j--)
                os.write(nibble(i >> (j << 2) & 0xf));

            os.writeString("\r\n");
            os.flush();
        }
    }
}


