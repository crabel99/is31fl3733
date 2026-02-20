#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "is31fl3733_color_utils.hpp"

// Manually create Wire1 on SERCOM1 with PA16 (SDA), PA17 (SCL)
// This allows hardware-specific I2C configuration independent of board variant
#ifndef NATIVE_TEST
static TwoWire Wire1(&sercom1, 16, 17); // PA16, PA17
#endif

namespace test_embedded {
namespace is31fl3733_pins {
// Pin mappings for test hardware
// ⚠️  ADJUST THESE TO MATCH YOUR ACTUAL HARDWARE WIRING
// Example hardware: Adafruit Feather M0 with IS31FL3733 breakout
// - INTB (interrupt): D11 → PA16
// - SDB (shutdown):   D12 → PA19
// Note: These differ from SimIO M0 (PA22/PA23) as those pins are not available on Feather M0
constexpr uint8_t INTB = 11; // IRQ pin - any available GPIO
constexpr uint8_t SDB = 12;  // Shutdown pin - any available GPIO

// I2C on SERCOM1: PA16 (physical pin 16), PA17 (physical pin 17)
// Note: Pin numbers are PA port numbers, not Arduino D numbers
constexpr uint8_t PIN_SDA = 16; // PA16
constexpr uint8_t PIN_SCL = 17; // PA17
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
#ifndef NATIVE_TEST
#ifdef __cplusplus
extern "C" {
#endif
void SERCOM1_Handler(void) {
	Wire1.onService();
}
#ifdef __cplusplus
}
#endif
#endif // NATIVE_TEST
