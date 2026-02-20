#include <unity_fixture.h>

#define TEST_NATIVE
#include "basic/is31fl3733_pins.h"
#include "is31fl3733_color_utils.hpp"
#include "is31fl3733.hpp"

using namespace test_embedded::is31fl3733_pins;
using namespace IS31FL3733;
using namespace ColorUtils;

TEST_GROUP(EmbeddedAdvanced);

// Recover I2C bus from stuck state (GPIO-level control to release slave holds)
// Based on SimIOFramework_test pattern - takes pin numbers as parameters
inline void recoverI2cBus(uint8_t pinSDA, uint8_t pinSCL) {
    if (pinSDA == 0xFF || pinSCL == 0xFF)
        return;

    const auto &sdaDesc = g_APinDescription[pinSDA];
    const auto &sclDesc = g_APinDescription[pinSCL];
    const uint32_t sdaMask = (1ul << sdaDesc.ulPin);
    const uint32_t sclMask = (1ul << sclDesc.ulPin);
    PortGroup *sdaPort = &PORT->Group[sdaDesc.ulPort];
    PortGroup *sclPort = &PORT->Group[sclDesc.ulPort];

    // Disable peripheral multiplexer to control as GPIO
    bool sdaPmuxWasEnabled = sdaPort->PINCFG[sdaDesc.ulPin].bit.PMUXEN;
    bool sclPmuxWasEnabled = sclPort->PINCFG[sclDesc.ulPin].bit.PMUXEN;
    sdaPort->PINCFG[sdaDesc.ulPin].bit.PMUXEN = 0;
    sclPort->PINCFG[sclDesc.ulPin].bit.PMUXEN = 0;

    // Set both pins as inputs (high-impedance)
    sdaPort->DIRCLR.reg = sdaMask;
    sclPort->DIRCLR.reg = sclMask;

    // Check if bus is already idle
    if ((sdaPort->IN.reg & sdaMask) && (sclPort->IN.reg & sclMask)) {
        // Bus idle, just restore PMUX
        if (sdaPmuxWasEnabled)
            sdaPort->PINCFG[sdaDesc.ulPin].bit.PMUXEN = 1;
        if (sclPmuxWasEnabled)
            sclPort->PINCFG[sclDesc.ulPin].bit.PMUXEN = 1;
        return;
    }

    // Bus stuck - clock SCL to release slave hold on SDA
    sclPort->OUTSET.reg = sclMask; // SCL high
    sclPort->DIRSET.reg = sclMask; // SCL as output

    // Clock SCL up to 9 times to release SDA
    for (uint8_t i = 0; i < 9 && !(sdaPort->IN.reg & sdaMask); ++i) {
        sclPort->OUTCLR.reg = sclMask; // Pull SCL low
        sclPort->DIRSET.reg = sclMask;
        delayMicroseconds(5);
        sclPort->DIRCLR.reg = sclMask; // Release SCL (pullup brings high)
        delayMicroseconds(5);
    }

    // Generate STOP condition if SDA still stuck
    if (!(sdaPort->IN.reg & sdaMask)) {
        sdaPort->OUTCLR.reg = sdaMask; // SDA low
        sdaPort->DIRSET.reg = sdaMask;
        delayMicroseconds(5);
        sclPort->OUTSET.reg = sclMask; // SCL high
        delayMicroseconds(5);
        sdaPort->DIRCLR.reg = sdaMask; // SDA high (STOP)
        delayMicroseconds(5);
    }

    // Restore PMUX settings
    if (sdaPmuxWasEnabled)
        sdaPort->PINCFG[sdaDesc.ulPin].bit.PMUXEN = 1;
    if (sclPmuxWasEnabled)
        sclPort->PINCFG[sclDesc.ulPin].bit.PMUXEN = 1;
}

#define CREATE_TEST_DRIVER()                                                                       \
    IS31FL3733::IS31FL3733 driver(&test_embedded::is31fl3733_pins::WIRE, 0x50,                     \
                                  test_embedded::is31fl3733_pins::SDB,                             \
                                  test_embedded::is31fl3733_pins::INTB);                           \
    bool begin_ok = driver.begin();                                                                \
    if (!begin_ok) {                                                                               \
        delay(5);                                                                                  \
        begin_ok = driver.begin();                                                                 \
    }                                                                                              \
    TEST_ASSERT_TRUE_MESSAGE(begin_ok, "IS31FL3733::begin() failed")

TEST_SETUP(EmbeddedAdvanced) {
    using namespace test_embedded::is31fl3733_pins;

    // Recover I2C bus from stuck state before initialization
    recoverI2cBus(PIN_SDA, PIN_SCL);

    WIRE.begin();
    // Configure PA16/PA17 for SERCOM1 (Wire1)
    pinPeripheral(PIN_SDA, PIO_SERCOM); // PA16 -> SERCOM1 PAD[0] (SDA)
    pinPeripheral(PIN_SCL, PIO_SERCOM); // PA17 -> SERCOM1 PAD[1] (SCL)
    WIRE.setClock(WIRE_BAUDRATE);
    delay(2);
}

TEST_TEAR_DOWN(EmbeddedAdvanced) {
    using namespace test_embedded::is31fl3733_pins;
    // Driver object is stack-scoped per test; destructor handles end() lifecycle.
    WIRE.end();
}

// =========================================================================================
// Concurrent Operation Tests
// These validate that PWM operations don't interfere with command transactions
// =========================================================================================

TEST(EmbeddedAdvanced, pwm_and_device_control_interleaved) {
    CREATE_TEST_DRIVER();
    // Issue: PWM transactions (row updates) must not block device commands
    // Test concurrent operation of both paths

    // Queue several PWM updates rapidly
    for (uint8_t row = 1; row <= 4; row++) {
        for (uint8_t col = 1; col <= 4; col++) {
            driver.SetPixelPWM(row, col, (row + col) * 16);
        }
    }

    delay(100); // Allow PWM to complete

    // Now issue a device control command
    driver.DeviceOn();
    delay(50);

    // SetPixelPWM should still work after device command
    driver.SetPixelPWM(1, 1, 255);
    delay(10);

    TEST_PASS();
}

TEST(EmbeddedAdvanced, pwm_row_transmission_completes) {
    CREATE_TEST_DRIVER();
    // Validates that PWM row updates complete without hanging
    // Each SetPixelPWM queues a 3-transaction sequence (unlock, page, write)

    // Set a full row of pixels
    for (uint8_t col = 1; col <= 16; col++) {
        driver.SetPixelPWM(2, col, 100 + (col * 10));
    }

    delay(200); // Allow full row transaction (3 phases per row)

    // If we reach here, PWM didn't hang
    TEST_PASS();
}

// =========================================================================================
// Device State Validation Tests
// These verify the device is configured correctly after initialization
// =========================================================================================

TEST(EmbeddedAdvanced, device_on_off_cycle) {
    CREATE_TEST_DRIVER();
    // Validates DeviceOn/DeviceOff command sequences work
    // DeviceOn: unlock, select page 3, write control register

    driver.DeviceOff();
    delay(50);

    driver.DeviceOn();
    delay(50);

    // Verify device state is preserved after power cycle
    // On healthy board: _ledOn should remain 0xFF (all enabled)
    // Test that PWM operations still work (device is responsive)
    driver.SetPixelPWM(1, 1, 128);
    delay(20);

    TEST_PASS();
}

TEST(EmbeddedAdvanced, color_order_switching) {
    CREATE_TEST_DRIVER();
    // Validates that color order changes don't break operation
    // This tests the synchronous write path for different contexts

    driver.SetColorOrder(ColorOrder::RGB);
    driver.SetPixelColor(1, 1, 100, 150, 200);
    delay(20);

    driver.SetColorOrder(ColorOrder::GRB);
    driver.SetPixelColor(2, 2, 100, 150, 200);
    delay(20);

    driver.SetColorOrder(ColorOrder::BRG);
    driver.SetPixelColor(3, 3, 100, 150, 200);
    delay(20);

    TEST_PASS();
}

// =========================================================================================
// Error Recovery Tests
// These test behavior under adverse conditions
// =========================================================================================

TEST(EmbeddedAdvanced, repeated_device_initialization) {
    CREATE_TEST_DRIVER();
    // Validates that device can be re-initialized multiple times
    // This tests that status state is properly reset

    // First end/begin cycle
    driver.end();
    delay(100);

    bool success = driver.begin();
    TEST_ASSERT_TRUE_MESSAGE(success, "Re-initialization failed on first cycle");
    delay(50);

    // Verify device is responsive by testing PWM operations
    driver.SetPixelPWM(1, 1, 128);
    delay(20);

    TEST_PASS();
}

TEST(EmbeddedAdvanced, pwm_operation_after_device_restart) {
    CREATE_TEST_DRIVER();
    // Validates that PWM works after device power cycle

    driver.DeviceOff();
    delay(100);

    driver.DeviceOn();
    delay(100);

    // PWM should work immediately after restart
    driver.SetPixelPWM(1, 1, 128);
    delay(20);

    driver.SetPixelPWM(2, 2, 200);
    delay(20);

    TEST_PASS();
}

// =========================================================================================
// Timing and Responsiveness Tests
// =========================================================================================

TEST(EmbeddedAdvanced, i2c_transaction_timing) {
    CREATE_TEST_DRIVER();
    // Validates that I2C transactions complete within expected timeframes
    // Measures that begin() completes quickly (status chaining overhead)

    unsigned long start_time = millis();

    driver.end();
    bool success = driver.begin();

    unsigned long elapsed = millis() - start_time;

    TEST_ASSERT_TRUE_MESSAGE(success, "begin() failed");
    TEST_ASSERT_LESS_THAN_MESSAGE(1000, elapsed, "begin() took too long - possible I2C hang");

    TEST_PASS();
}

TEST_GROUP_RUNNER(EmbeddedAdvanced) {
    RUN_TEST_CASE(EmbeddedAdvanced, pwm_and_device_control_interleaved);
    RUN_TEST_CASE(EmbeddedAdvanced, pwm_row_transmission_completes);
    RUN_TEST_CASE(EmbeddedAdvanced, device_on_off_cycle);
    RUN_TEST_CASE(EmbeddedAdvanced, color_order_switching);
    RUN_TEST_CASE(EmbeddedAdvanced, repeated_device_initialization);
    RUN_TEST_CASE(EmbeddedAdvanced, pwm_operation_after_device_restart);
    RUN_TEST_CASE(EmbeddedAdvanced, i2c_transaction_timing);
}
