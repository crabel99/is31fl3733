/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Cal Abel. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *
 *  Based on the original IS31FL3733 C++ library by Neil Enns
 *  (https://github.com/neilenns/is31fl3733).
 *  RGB matrix helper - part of async DMA rewrite for Arduino SAMD.
 *
 *  IS31FL3733 RGB matrix helper utilities.
 *--------------------------------------------------------------------------------------------*/
#pragma once

#include "is31fl3733.hpp"
#include "is31fl3733_color_utils.hpp"

namespace IS31FL3733 {
/// @brief Convenience RGB matrix API built on top of the core async driver.
///
/// Inherits all base driver features (begin/end, ABM, raw pixel/mode control)
/// and adds higher-level color helpers for 32-bit packed RGB and HSV.
class IS31FL3733RgbMatrix : public IS31FL3733 {
  public:
    /** @name Constants */
    /** @{ */
    /// @brief Logical RGB matrix row count.
    static constexpr uint8_t kRows = 4;
    /// @brief Logical RGB matrix column count.
    static constexpr uint8_t kCols = 16;
    /** @} */

    /** @name Constructor */
    /** @{ */
    /// @brief Construct RGB matrix helper with same arguments as base driver.
    /// @param wire Pointer to TwoWire I2C interface.
    /// @param addr 7-bit I2C address (typically 0x50).
    /// @param sdbPin Optional SDB (shutdown) pin (active high). Use 0xFF to disable.
    /// @param irqPin Optional IRQ pin for open/short detection. Use 0xFF to disable.
    /// @param order Initial logical RGB channel order.
    IS31FL3733RgbMatrix(TwoWire *wire, uint8_t addr = 0x50, uint8_t sdbPin = 0xFF,
                        uint8_t irqPin = 0xFF, ColorOrder order = ColorOrder::GRB)
        : IS31FL3733(wire, addr, sdbPin, irqPin) {
        SetColorOrder(order);
    }
    /** @} */

    using IS31FL3733::GetColorOrder;
    using IS31FL3733::SetColorOrder;

    /** @name Validation Helpers */
    /** @{ */
    /// @brief Validate a logical RGB row index.
    /// @param row Logical row (1..kRows).
    /// @return True if row is in range.
    static bool IsValidRow(uint8_t row) {
        return row >= 1 && row <= kRows;
    }

    /// @brief Validate a logical RGB column index.
    /// @param col Logical column (1..kCols).
    /// @return True if column is in range.
    static bool IsValidCol(uint8_t col) {
        return col >= 1 && col <= kCols;
    }
    /** @} */

    /** @name Color Helpers */
    /** @{ */
    /// @brief Set a logical RGB pixel from packed 0xRRGGBB value.
    /// @param row Logical row (1..4).
    /// @param col Logical column (1..16).
    /// @param packedRgb Packed RGB color (0xRRGGBB).
    /// @param gamma If true, applies Gamma32() before writing.
    void SetPixelColor32(uint8_t row, uint8_t col, uint32_t packedRgb, bool gamma = false) {
        if (!IsValidRow(row) || !IsValidCol(col)) {
            return;
        }

        uint32_t color = gamma ? ColorUtils::Gamma32(packedRgb) : packedRgb;

        const uint8_t r = static_cast<uint8_t>((color >> 16) & 0xFF);
        const uint8_t g = static_cast<uint8_t>((color >> 8) & 0xFF);
        const uint8_t b = static_cast<uint8_t>(color & 0xFF);

        IS31FL3733::SetPixelColor(row, col, r, g, b);
    }

    /// @brief Set a logical RGB pixel from HSV.
    /// @param row Logical row (1..4).
    /// @param col Logical column (1..16).
    /// @param hue Hue (0..65535).
    /// @param sat Saturation (0..255).
    /// @param val Value (0..255).
    /// @param gamma If true, applies Gamma32() after HSV conversion.
    void SetPixelHSV(uint8_t row, uint8_t col, uint16_t hue, uint8_t sat = 255, uint8_t val = 255,
                     bool gamma = true) {
        uint32_t color = ColorUtils::ColorHSV(hue, sat, val);
        SetPixelColor32(row, col, color, gamma);
    }

    /// @brief Fill logical matrix with packed RGB color.
    /// @param packedRgb Packed RGB color (0xRRGGBB).
    /// @param gamma If true, applies Gamma32() before writing.
    void FillColor32(uint32_t packedRgb, bool gamma = false) {
        for (uint8_t row = 1; row <= kRows; row++) {
            for (uint8_t col = 1; col <= kCols; col++) {
                SetPixelColor32(row, col, packedRgb, gamma);
            }
        }
    }

    /// @brief Fill logical matrix with HSV color.
    /// @param hue Hue (0..65535).
    /// @param sat Saturation (0..255).
    /// @param val Value (0..255).
    /// @param gamma If true, applies Gamma32() after HSV conversion.
    void FillHSV(uint16_t hue, uint8_t sat = 255, uint8_t val = 255, bool gamma = true) {
        const uint32_t color = ColorUtils::ColorHSV(hue, sat, val);
        FillColor32(color, gamma);
    }
    /** @} */
};
} // namespace IS31FL3733
