#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <wiring_private.h>

#include "is31fl3733_color_utils.hpp"

// Wire1 is pre-defined by the SimIO Device M0 variant on SERCOM1 (PA16/PA17).
// No manual TwoWire construction needed.

namespace test_embedded {
namespace is31fl3733_pins {
// Pin mappings aligned with SimIODevice trim-wheel board map
// (`src/SimIO/SimIODevice/include/boards/trim_wheel.h`):
// - INTB (interrupt): D34 → PA14
// - SDB (shutdown):   D36 → PA27
// - SDA: D18 → PA16 (Wire1, SERCOM1 PAD[0])
// - SCL: D19 → PA17 (Wire1, SERCOM1 PAD[1])
constexpr uint8_t INTB = 34; // PA14, Arduino pin 34
constexpr uint8_t SDB = 36;  // PA27, Arduino pin 36

// Wire1 SDA/SCL pin numbers (Arduino indices matching PIN_WIRE1_SDA/SCL in variant)
constexpr uint8_t PIN_SDA = PIN_WIRE1_SDA; // 18 → PA16
constexpr uint8_t PIN_SCL = PIN_WIRE1_SCL; // 19 → PA17
constexpr uint32_t WIRE_BAUDRATE = 400000UL;
static TwoWire &WIRE = Wire1;

// IS31FL3733 matrix wiring for BL51 bargraph.
// Green rows: G1..G4 -> SW1, SW4, SW7, SW10
// Red rows:   R1..R4 -> SW2, SW5, SW8, SW11
// Blue rows SW3/SW6/SW9/SW12 are intentionally unused.
constexpr uint8_t SW_GREEN[4] = {1, 4, 7, 10};
constexpr uint8_t SW_RED[4] = {2, 5, 8, 11};
constexpr uint8_t CS_FIRST = 1;
constexpr uint8_t CS_LAST = 13;
constexpr uint8_t CS_COUNT = 13;
constexpr uint8_t BARGRAPH_GROUPS = 4;
constexpr uint8_t BARGRAPH_SEGMENTS = 51;

struct LedAddress {
    uint8_t sw;
    uint8_t cs;
};

struct SegmentAddressPair {
    LedAddress green;
    LedAddress red;
};

inline bool IsValidSegment(uint8_t segment) {
    return segment >= 1 && segment <= BARGRAPH_SEGMENTS;
}

inline LedAddress SegmentToLedAddress(uint8_t segment, IS31FL3733::ColorUtils::LedColor color) {
    const uint8_t zeroBased = segment - 1;
    const uint8_t group = zeroBased % BARGRAPH_GROUPS;
    const uint8_t column = (zeroBased / BARGRAPH_GROUPS) + CS_FIRST;

    LedAddress address{};
    address.cs = column;
    address.sw =
        (color == IS31FL3733::ColorUtils::LedColor::Green) ? SW_GREEN[group] : SW_RED[group];
    return address;
}

inline SegmentAddressPair SegmentToPair(uint8_t segment) {
    SegmentAddressPair pair{};
    pair.green = SegmentToLedAddress(segment, IS31FL3733::ColorUtils::LedColor::Green);
    pair.red = SegmentToLedAddress(segment, IS31FL3733::ColorUtils::LedColor::Red);
    return pair;
}
} // namespace is31fl3733_pins
} // namespace test_embedded

// SERCOM1 interrupt handler for Wire1
// Required when manually creating TwoWire object with custom SERCOM
// #ifndef NATIVE_TEST
// #ifdef __cplusplus
// extern "C" {
// #endif
// void SERCOM1_Handler(void) {
//     Wire1.onService();
// }
// #ifdef __cplusplus
// }
// #endif
// #endif // NATIVE_TEST
