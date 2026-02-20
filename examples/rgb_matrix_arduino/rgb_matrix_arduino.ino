/**
 * @file rgb_matrix_arduino.ino
 * @author Cal Abel
 * @brief Demonstrates how to use the IS31FL3733RgbMatrix convenience class for high-level
 * color control with HSV, 32-bit RGB, and gamma correction on an Arduino.
 * @version 2.0.0
 * @date 2026-02-19
 *
 * @copyright Copyright 2026
 *
 */
/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Cal Abel. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *--------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>

#include "is31fl3733_rgb_matrix.hpp"

using namespace IS31FL3733;

// Arduino pin for the SDB line which is set high to enable the IS31FL3733 chip.
const uint8_t SDB_PIN = 4;
// Arduino pin for the IS13FL3733 interrupt pin (optional, 0xFF to disable).
const uint8_t INTB_PIN = 0xFF;
const uint8_t IS31_ADDR = 0x50;

// Create RGB matrix helper (inherits all base driver features + adds color convenience).
IS31FL3733RgbMatrix matrix(&Wire, IS31_ADDR, SDB_PIN, INTB_PIN, ColorOrder::RGB);

// Simple animation state.
uint16_t hueOffset = 0;

/**
 * @brief Arduino initialization.
 *
 */
void setup() {
    // Initialize serial and I2C.
    Serial.begin(115200);
    Wire.begin();

    // Wait for serial to connect so the debug output can be seen.
    while (!Serial)
        ; // Waiting for Serial Monitor

    Serial.println("\n=== IS31FL3733 RGB Matrix Example ===");
    Serial.print("Driver address: 0x");
    Serial.println(IS31_ADDR, HEX);

    // Initialize matrix with default GCC and clear to black.
    if (!matrix.begin()) {
        Serial.println("matrix.begin() failed!");
        while (true)
            ; // Halt
    }

    Serial.println("Matrix initialized");
    delay(1000);

    // Demo 1: Fill with solid 32-bit color (red, with gamma correction).
    Serial.println("Demo 1: Solid red with gamma correction");
    matrix.FillColor32(0xFF0000, true);
    delay(2000);

    // Demo 2: Fill with HSV color (cyan, with gamma correction).
    Serial.println("Demo 2: Solid cyan (HSV) with gamma correction");
    matrix.FillHSV(32768, 255, 255, true);
    delay(2000);

    // Demo 3: Individual pixel coloring.
    Serial.println("Demo 3: Corner pixels (no gamma)");
    matrix.FillColor32(0x000000);            // Clear to black
    matrix.SetPixelColor32(1, 1, 0xFF0000);  // Top-left: Red
    matrix.SetPixelColor32(1, 16, 0x00FF00); // Top-right: Green
    matrix.SetPixelColor32(4, 1, 0x0000FF);  // Bottom-left: Blue
    matrix.SetPixelColor32(4, 16, 0xFFFF00); // Bottom-right: Yellow
    delay(2000);

    // Demo 4: HSV rainbow row.
    Serial.println("Demo 4: HSV rainbow gradient across row 2");
    matrix.FillColor32(0x000000); // Clear to black
    for (uint8_t col = 1; col <= 16; col++) {
        uint16_t hue = (col - 1) * (65535 / 16);
        matrix.SetPixelHSV(2, col, hue, 255, 255, true);
    }
    delay(2000);

    Serial.println("Starting rainbow animation...");
}

static uint32_t lastUpdate = 0;
const uint32_t updateInterval = 50; // 50ms = ~20 FPS
/**
 * @brief Arduino loop - animates a rainbow wave across the matrix.
 *
 */
void loop() {
    if (millis() - lastUpdate >= updateInterval) {
        lastUpdate = millis();

        // Rainbow wave animation across entire 4x16 matrix.
        for (uint8_t row = 1; row <= matrix.kRows; row++) {
            for (uint8_t col = 1; col <= matrix.kCols; col++) {
                // Calculate hue based on position and animation offset.
                uint16_t hue = hueOffset + (row * 4096) + (col * 2048);
                matrix.SetPixelHSV(row, col, hue, 255, 200, true);
            }
        }

        // Advance animation.
        hueOffset += 512;
    }
}
