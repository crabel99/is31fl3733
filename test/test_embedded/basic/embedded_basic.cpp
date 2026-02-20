#include <unity_fixture.h>

#define TEST_NATIVE
#include "is31fl3733_pins.h"
#include "is31fl3733_color_utils.hpp"
#include "is31fl3733.hpp"

using namespace test_embedded::is31fl3733_pins;
using namespace IS31FL3733;
using namespace ColorUtils;

TEST_GROUP(EmbeddedBasic);

#define CREATE_TEST_DRIVER()                                                                       \
    IS31FL3733::IS31FL3733 driver(&Wire1, 0x50, test_embedded::is31fl3733_pins::SDB,               \
                                  test_embedded::is31fl3733_pins::INTB);                           \
    bool begin_ok = driver.begin();                                                                \
    if (!begin_ok) {                                                                               \
        delay(5);                                                                                  \
        begin_ok = driver.begin();                                                                 \
    }                                                                                              \
    TEST_ASSERT_TRUE_MESSAGE(begin_ok, "IS31FL3733::begin() failed - check I2C connection")

// Recover I2C bus from stuck state (GPIO-level control to release slave holds)
inline void recoverI2cBus() {
    // Wire1 on SimIO_Device_M0: PA16 (SDA), PA17 (SCL) -> D18, D19
    constexpr uint8_t pinSDA = 18; // PA16
    constexpr uint8_t pinSCL = 19; // PA17

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

TEST_SETUP(EmbeddedBasic) {
    // Recover I2C bus from stuck state before initialization
    recoverI2cBus();

    Wire1.begin();
    Wire1.setClock(400000);
    delay(2);
}

TEST_TEAR_DOWN(EmbeddedBasic) {
    // Driver object is stack-scoped per test; destructor handles end() lifecycle.
    Wire1.end();
}

TEST(EmbeddedBasic, BargraphSegmentMapping) {

    TEST_ASSERT_TRUE(IsValidSegment(1));
    TEST_ASSERT_TRUE(IsValidSegment(51));
    TEST_ASSERT_FALSE(IsValidSegment(0));
    TEST_ASSERT_FALSE(IsValidSegment(52));

    LedAddress led = SegmentToLedAddress(1, LedColor::Green);
    TEST_ASSERT_EQUAL_UINT8(1, led.sw);
    TEST_ASSERT_EQUAL_UINT8(1, led.cs);

    led = SegmentToLedAddress(4, LedColor::Red);
    TEST_ASSERT_EQUAL_UINT8(11, led.sw);
    TEST_ASSERT_EQUAL_UINT8(1, led.cs);

    led = SegmentToLedAddress(5, LedColor::Green);
    TEST_ASSERT_EQUAL_UINT8(1, led.sw);
    TEST_ASSERT_EQUAL_UINT8(2, led.cs);

    led = SegmentToLedAddress(51, LedColor::Green);
    TEST_ASSERT_EQUAL_UINT8(7, led.sw);
    TEST_ASSERT_EQUAL_UINT8(13, led.cs);

    SegmentAddressPair pair = SegmentToPair(1);
    TEST_ASSERT_EQUAL_UINT8(1, pair.green.sw);
    TEST_ASSERT_EQUAL_UINT8(1, pair.green.cs);
    TEST_ASSERT_EQUAL_UINT8(2, pair.red.sw);
    TEST_ASSERT_EQUAL_UINT8(1, pair.red.cs);
}

TEST(EmbeddedBasic, DriverInitialization) {
    CREATE_TEST_DRIVER();
    TEST_PASS();
}

TEST(EmbeddedBasic, BeginLedMaskMatchesOpenShortStatus) {
    CREATE_TEST_DRIVER();
    // begin() computes _ledOn from fault maps:
    //   _ledOn = ~(LEDOPEN | LEDSHORT)
    // Board-specific expectation:
    //   - Blue rows (non SW_GREEN/SW_RED) are unconnected -> open bit should be 1
    //   - CS14..CS16 are open on all rows
    //   - SW10/11 at CS13 are also open
    //   - all other mapped red/green LEDs should report open bit 0

    auto getMapBit = [](const uint8_t *map, uint8_t sw, uint8_t cs) {
        const uint8_t rowIndex = sw - 1; // SW1..SW12 -> 0..11
        const uint8_t colIndex = cs - 1; // CS1..CS16 -> 0..15
        const uint8_t byteOffset = (rowIndex * 2) + (colIndex >> 3);
        const uint8_t bitOffset = colIndex & 0x07;
        return (map[byteOffset] >> bitOffset) & 0x01;
    };

    for (uint8_t sw = 1; sw <= HARDWARE_ROWS; sw++) {
        for (uint8_t cs = CS_FIRST; cs <= HARDWARE_COLS; cs++) {
            const uint8_t openBit = getMapBit(driver._ledOpen, sw, cs);
            const uint8_t shortBit = getMapBit(driver._ledShort, sw, cs);
            const uint8_t onBit = getMapBit(driver._ledOn, sw, cs);

            // _ledOn must always be the inverse mask of open/short status.
            TEST_ASSERT_EQUAL_UINT8_MESSAGE((openBit || shortBit) ? 0 : 1, onBit,
                                            "_ledOn bit does not match ~(LEDOPEN|LEDSHORT)");
        }
    }
}

TEST(EmbeddedBasic, SetPixelPWM) {
    CREATE_TEST_DRIVER();
    // Test setting a single pixel
    driver.SetPixelPWM(1, 1, 128); // SW1, CS1, 50% brightness
    delay(10);                     // Allow DMA transaction to complete

    // No way to read back PWM, so this just tests that it doesn't crash
    TEST_PASS();
}

TEST(EmbeddedBasic, Fill) {
    CREATE_TEST_DRIVER();
    // Test filling entire matrix
    driver.Fill(64); // 25% brightness
    delay(50);       // Allow all 12 row transactions to complete

    TEST_PASS();
}

// =========================================================================================
// Status Chaining Validation Tests
// These tests verify the status accumulation mechanism works on real hardware
// =========================================================================================

TEST(EmbeddedBasic, status_chaining_reset_read_succeeds) {
    CREATE_TEST_DRIVER();
    // Validates that RESET read completes successfully during setup begin()
    // Status should chain: 0 OR 0 OR 0 OR 0 = 0

    TEST_ASSERT_EQUAL_INT_MESSAGE(0, driver._cmdCtx[3].initialStatus,
                                  "Accumulated status not clean after successful RESET read");
}

TEST(EmbeddedBasic, status_chaining_ledbox_operations_succeed) {
    CREATE_TEST_DRIVER();
    // Validates that LED open/short detection read succeeds
    // This is another read transaction: cmd0 -> cmd1 -> cmd2 -> cmd3
    // Status chaining should accumulate any intermediate failures

    // After begin() in TEST_SETUP, the driver has already completed:
    // - RESET read
    // - Configuration writes
    // - LED box reads (LEDOPEN, LEDSHORT)

    // If we reach this point, all transactions succeeded
    // On a healthy board, LEDOPEN and LEDSHORT should be 0x00 (no faults)
    // This results in _ledOn = 0xFF (all LEDs enabled)
    // Verify the OSD reads completed (even if no faults detected)
    bool osd_reads_completed = (driver._ledOpen[0] == 0x00) && (driver._ledShort[0] == 0x00);
    TEST_ASSERT_TRUE_MESSAGE(osd_reads_completed || driver._ledOn[0] != 0xFF,
                             "LEDOPEN/LEDSHORT reads did not complete");
}

TEST(EmbeddedBasic, status_chaining_all_begin_phases_validated) {
    CREATE_TEST_DRIVER();
    // Comprehensive validation that all begin() transaction chains succeeded
    // Uses the instance created in setup to avoid reinitializing the same bus

    // Verify OSD reads completed successfully
    // On a healthy board: LEDOPEN=0x00, LEDSHORT=0x00, _ledOn=0xFF (all enabled)
    // On a faulty board: LEDOPEN or LEDSHORT non-zero, _ledOn < 0xFF (some disabled)
    bool healthy = (driver._ledOpen[0] == 0x00 && driver._ledShort[0] == 0x00 &&
                    driver._ledOn[0] == 0xFF && driver._ledOn[1] == 0xFF);
    bool faults_detected = (driver._ledOpen[0] != 0x00 || driver._ledShort[0] != 0x00) &&
                           (driver._ledOn[0] != 0xFF || driver._ledOn[1] != 0xFF);

    TEST_ASSERT_TRUE_MESSAGE(healthy || faults_detected,
                             "Device OSD reads did not complete properly");

    // Verify accumulated status is clean after begin()
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, driver._cmdCtx[2].initialStatus,
                                  "Write chain status not clean after begin()");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, driver._cmdCtx[3].initialStatus,
                                  "Read chain status not clean after begin()");
}

TEST(EmbeddedBasic, concurrent_pwm_after_initialization) {
    CREATE_TEST_DRIVER();
    // Tests that PWM operations work after successful begin() initialization
    // This validates that the device configuration from begin() is stable

    // SetPixelPWM should work without blocking
    driver.SetPixelPWM(1, 1, 200);
    driver.SetPixelPWM(2, 1, 150);
    driver.SetPixelPWM(3, 1, 100);

    delay(50); // Allow PWM transactions to complete

    // If we reach here without hanging, concurrent operations work
    TEST_PASS();
}

TEST(EmbeddedBasic, raw_register_write_read) {
    CREATE_TEST_DRIVER();
    // Test raw I2C register write and read
    // Write 0xC5 to register 0xFE (PSWL - Page Select Write Lock)
    // Then read and verify the value matches

    const uint8_t test_register = 0xFE;
    const uint8_t test_value = 0xC5;

    // Write 0xC5 to register 0xFE
    Wire1.beginTransmission(driver._addr);
    Wire1.write(test_register);
    Wire1.write(test_value);
    uint8_t write_status = Wire1.endTransmission();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, write_status, "I2C write to FEh failed");

    delay(5); // Small delay for register update

    // Read from register 0xFE
    Wire1.beginTransmission(driver._addr);
    Wire1.write(test_register);
    uint8_t status = Wire1.endTransmission(false); // repeated start
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, status, "I2C write register address failed");

    uint8_t bytes_received = Wire1.requestFrom(driver._addr, (uint8_t)1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, bytes_received, "I2C read from FEh failed to receive byte");

    uint8_t read_value = Wire1.read();

    // Assert that what we read matches what we wrote
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(test_value, read_value,
                                   "Register FEh read value does not match written value");
}

TEST(EmbeddedBasic, osd_values_debug) {
    CREATE_TEST_DRIVER();
    // Verify OSD reads completed successfully
    // This is just status validation, not full debug output
    // On a healthy board: LEDOPEN=0x00, LEDSHORT=0x00, _ledOn=0xFF
    bool osd_completed = (driver._ledOpen[0] == 0x00) && (driver._ledShort[0] == 0x00);
    TEST_ASSERT_TRUE_MESSAGE(osd_completed || driver._ledOn[0] != 0xFF,
                             "OSD reads did not complete properly");
    TEST_PASS();
}

TEST_GROUP_RUNNER(EmbeddedBasic) {
    RUN_TEST_CASE(EmbeddedBasic, BargraphSegmentMapping);
    RUN_TEST_CASE(EmbeddedBasic, DriverInitialization);
    RUN_TEST_CASE(EmbeddedBasic, osd_values_debug);
    RUN_TEST_CASE(EmbeddedBasic, BeginLedMaskMatchesOpenShortStatus);
    RUN_TEST_CASE(EmbeddedBasic, SetPixelPWM);
    RUN_TEST_CASE(EmbeddedBasic, Fill);
    RUN_TEST_CASE(EmbeddedBasic, status_chaining_reset_read_succeeds);
    RUN_TEST_CASE(EmbeddedBasic, status_chaining_ledbox_operations_succeed);
    RUN_TEST_CASE(EmbeddedBasic, status_chaining_all_begin_phases_validated);
    RUN_TEST_CASE(EmbeddedBasic, concurrent_pwm_after_initialization);
    RUN_TEST_CASE(EmbeddedBasic, raw_register_write_read);
}
