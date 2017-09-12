#include <teraranger/helper_lib.h>

namespace teraranger
{
    uint8_t HelperLib::crc8(uint8_t *p, uint8_t len)
    {
        uint16_t i;
        uint16_t crc = 0x0;

        while (len--)
        {
            i = (crc ^ *p++) & 0xFF;
            crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
        }
        return crc & 0xFF;
    }

    float HelperLib::two_chars_to_float(uint8_t c1, uint8_t c2)
    {
        int16_t current_range = c1 << 8;
        current_range |= c2;

        float res = (float)current_range;
        return res;
    }
} //namespace teraranger
