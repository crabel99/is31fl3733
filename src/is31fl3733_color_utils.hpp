/*---------------------------------------------------------------------------------------------
 *  This file incorporates code from the Adafruit_NeoPixel library:
 *  https://github.com/adafruit/Adafruit_NeoPixel
 *
 *  Portions derived from Adafruit_NeoPixel are Copyright (c) Adafruit Industries.
 *  Licensed under the GNU Lesser General Public License v3.0 (LGPL-3.0).
 *  See: https://www.gnu.org/licenses/lgpl-3.0.html
 *--------------------------------------------------------------------------------------------*/
#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#endif

namespace IS31FL3733 {
namespace ColorUtils {
enum class LedColor : uint8_t {
    Green,
    Red,
    Blue,
};

#ifdef ARDUINO
#define IS31FL3733_PROGMEM PROGMEM
#else
#define IS31FL3733_PROGMEM
#endif

static const uint8_t IS31FL3733_PROGMEM kGammaTable[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,
    2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,
    5,   5,   6,   6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,  11,
    11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,
    20,  20,  21,  21,  22,  22,  23,  24,  24,  25,  25,  26,  27,  27,  28,  29,  29,  30,  31,
    31,  32,  33,  34,  34,  35,  36,  37,  38,  38,  39,  40,  41,  42,  42,  43,  44,  45,  46,
    47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,
    66,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,  80,  81,  82,  84,  85,  86,  88,  89,
    90,  92,  93,  94,  96,  97,  99,  100, 102, 103, 105, 106, 108, 109, 111, 112, 114, 115, 117,
    119, 120, 122, 124, 125, 127, 129, 130, 132, 134, 136, 137, 139, 141, 143, 145, 146, 148, 150,
    152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186, 188,
    191, 193, 195, 197, 199, 202, 204, 206, 209, 211, 213, 215, 218, 220, 223, 225, 227, 230, 232,
    235, 237, 240, 242, 245, 247, 250, 252, 255};

inline uint8_t Gamma8(uint8_t x) {
#ifdef ARDUINO
    return pgm_read_byte(&kGammaTable[x]);
#else
    return kGammaTable[x];
#endif
}

inline uint32_t Gamma32(uint32_t x) {
    uint8_t *bytes = reinterpret_cast<uint8_t *>(&x);
    for (uint8_t i = 0; i < 4; i++)
        bytes[i] = Gamma8(bytes[i]);

    return x;
}

inline uint32_t PackRGB(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

inline uint32_t ColorHSV(uint16_t hue, uint8_t sat = 255, uint8_t val = 255) {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    hue = (uint32_t)(hue * 1530UL + 32768UL) / 65536UL;

    if (hue < 510) {
        b = 0;
        if (hue < 255) {
            r = 255;
            g = hue;
        } else {
            r = 510 - hue;
            g = 255;
        }
    } else if (hue < 1020) {
        r = 0;
        if (hue < 765) {
            g = 255;
            b = hue - 510;
        } else {
            g = 1020 - hue;
            b = 255;
        }
    } else if (hue < 1530) {
        g = 0;
        if (hue < 1275) {
            r = hue - 1020;
            b = 255;
        } else {
            r = 255;
            b = 1530 - hue;
        }
    } else {
        r = 255;
        g = 0;
        b = 0;
    }

    const uint32_t v1 = 1 + val;
    const uint16_t s1 = 1 + sat;
    const uint8_t s2 = 255 - sat;

    return (((((r * s1) >> 8) + s2) * v1) & 0xff00) << 8 |
           (((((g * s1) >> 8) + s2) * v1) & 0xff00) | (((((b * s1) >> 8) + s2) * v1) >> 8);
}

#undef IS31FL3733_PROGMEM
} // namespace ColorUtils
} // namespace IS31FL3733
