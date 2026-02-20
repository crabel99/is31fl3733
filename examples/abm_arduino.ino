/**
 * @file abm_arduino.ino
 * @author Neil Enns (neile@live.com) - Original example
 * @author Cal Abel - Updated for async driver APIs
 * @brief Demonstrates how to use the IS31FL3733 chip to enable ABM mode for all LEDs using an
 * Arduino with interrupt detection of when ABM completes.
 * @version 2.0.0
 * @date 2026-02-19
 *
 * @copyright Copyright 2021 Neil Enns, 2026 Cal Abel
 *
 */
/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Neil Enns, Cal Abel. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *
 *  Based on original example by Neil Enns (https://github.com/neilenns/is31fl3733).
 *  Updated for asynchronous DMA-driven IS31FL3733 driver.
 *--------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>

#include "is31fl3733.hpp"

using namespace IS31FL3733;

// Arduino pin for the SDB line which is set high to enable the IS31FL3733 chip.
const uint8_t SDB_PIN = 4;
// Arduino pin for the IS13FL3733 interrupt pin.
const uint8_t INTB_PIN = 3;
const uint8_t IS31_ADDR = 0x50;

// Create a new base async driver instance.
::IS31FL3733::IS31FL3733 driver(&Wire, IS31_ADDR, SDB_PIN, INTB_PIN);

/**
 * @brief Finite state machine states for the LEDs.
 *
 */
enum LedState {
    ABMNotStarted, //< Before ABM starts running.
    ABMRunning,    //< While ABM is running.
    ABMComplete,   //< After the ABM complete interrupt fires.
    LEDOn          //< ABM is complete and LEDs are on.
};

/**
 * @brief Current state of the finite state machine.
 *
 */
volatile auto ledState = LedState::ABMNotStarted;

/**
 * @brief Interrupt handler for when ABM finishes.
 *
 */
void abm_completed() {
    ledState = LedState::ABMComplete;
}

/**
 * @brief Arduino initialization.
 *
 */
void setup() {
    // Initialize serial and I2C.
    Serial.begin(115200);
    Wire.begin();

    // Register ABM completion callback.
    driver.SetABMCallback(1, abm_completed);

    // Wait for serial to connect so the debug output can be seen.
    while (!Serial)
        ; // Waiting for Serial Monitor

    Serial.print("\nIS31FL3733B test of driver at address 0x");
    Serial.println(IS31_ADDR, HEX);
    Serial.println("Waiting 5 seconds to begin.");

    // This gives enough time to start up a connected logic analyzer
    delay(5000);

    Serial.println("Initializing");
    if (!driver.begin()) {
        Serial.println("driver.begin() failed");
        while (true)
            ;
    }

    Serial.println("Setting global current control to half");
    driver.SetGCC(127);

    Serial.println("Setting PWM state for all LEDs to half power");
    driver.Fill(128);

    Serial.println("Setting state of all LEDs to ON");
    Serial.println("Configure all LEDs for ABM1");
    driver.SetMatrixMode(ABMMode::ABM1);

    ABMConfig ABM1;

    ABM1.T1 = T1_840MS;
    ABM1.T2 = T2_840MS;
    ABM1.T3 = T3_840MS;
    ABM1.T4 = T4_840MS;
    ABM1.Tbegin = LOOP_BEGIN_T4;
    ABM1.Tend = LOOP_END_T3;
    ABM1.Times = 2;

    // Write ABM structure parameters to device registers.
    driver.ConfigureABM1(ABM1);

    // Enable interrupts when ABM completes and auto-clear them after 8ms
    driver.SetIMR(IMR_IAB);

    // Start ABM mode operation.
    driver.EnableABM(true);
    driver.TriggerABM();
    ledState = LedState::ABMRunning;
}

static uint32_t lastPrint = 0;
static LedState lastState = LedState::ABMNotStarted;
const uint32_t printInterval = 500;

/**
 * @brief Arduino loop.
 *
 */
void loop() { // Simple finite state machine to switch LEDs on after ABM finishes running.
    switch (ledState) {
    case LedState::ABMNotStarted: {
        if (ledState != lastState || millis() - lastPrint >= printInterval) {
            Serial.println("ABM not started");
            lastPrint = millis();
            lastState = ledState;
        }
        break;
    }
    case LedState::ABMRunning: {
        if (ledState != lastState || millis() - lastPrint >= printInterval) {
            Serial.println("ABM running");
            lastPrint = millis();
            lastState = ledState;
        }
        break;
    }
    case LedState::ABMComplete: {
        Serial.println("ABM1 completed");
        Serial.println("Configure all LEDs for full on");
        driver.EnableABM(false);
        driver.SetMatrixMode(ABMMode::PWM);

        ledState = LedState::LEDOn;
        lastState = ledState;
        break;
    }
    case LedState::LEDOn: {
        if (ledState != lastState || millis() - lastPrint >= printInterval) {
            Serial.println("LEDs on");
            lastPrint = millis();
            lastState = ledState;
        }
        break;
    }
    }
}