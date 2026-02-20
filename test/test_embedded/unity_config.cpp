#include "unity_config.h"
#include <debug.h>

extern "C"
{
    void unity_output_char(char c)
    {
        SERIAL_PORT.write(static_cast<uint8_t>(c));
    }

    void unity_output_start(void)
    {
        SERIAL_PORT.begin();
    }

    void unity_output_complete(void)
    {
        SERIAL_PORT.flush();
    }
}
