#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "is31fl3733_color_utils.hpp"

namespace test_embedded
{
namespace is31fl3733_pins
{
constexpr uint8_t INTB = 16;
constexpr uint8_t SDB = 17;
// SimIO_Device_M0 variant mapping for Wire1: PA16 -> D18, PA17 -> D19.
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

struct LedAddress
{
	uint8_t sw;
	uint8_t cs;
};

struct SegmentAddressPair
{
	LedAddress green;
	LedAddress red;
};

inline bool IsValidSegment(uint8_t segment)
{
	return segment >= 1 && segment <= BARGRAPH_SEGMENTS;
}

inline LedAddress SegmentToLedAddress(uint8_t segment, IS31FL3733::ColorUtils::LedColor color)
{
	const uint8_t zeroBased = segment - 1;
	const uint8_t group = zeroBased % BARGRAPH_GROUPS;
	const uint8_t column = (zeroBased / BARGRAPH_GROUPS) + CS_FIRST;

	LedAddress address{};
	address.cs = column;
	address.sw = (color == IS31FL3733::ColorUtils::LedColor::Green) ? SW_GREEN[group] : SW_RED[group];
	return address;
}

inline SegmentAddressPair SegmentToPair(uint8_t segment)
{
	SegmentAddressPair pair{};
	pair.green = SegmentToLedAddress(segment, IS31FL3733::ColorUtils::LedColor::Green);
	pair.red = SegmentToLedAddress(segment, IS31FL3733::ColorUtils::LedColor::Red);
	return pair;
}
} // namespace is31fl3733_pins
} // namespace test_embedded
