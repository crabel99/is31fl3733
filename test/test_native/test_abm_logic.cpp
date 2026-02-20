/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Cal Abel. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *
 *  Native unit tests for IS31FL3733 ABM (Auto Breath Mode) async driver.
 *  Focuses on non-hardware-dependent logic: initialization, bitfield accessors, and state.
 *--------------------------------------------------------------------------------------------*/

#define TEST_NATIVE

#include <unity_fixture.h>
#include "is31fl3733.hpp"

using namespace IS31FL3733;
using Driver = ::IS31FL3733::IS31FL3733;

TEST_GROUP(ABMAsyncLogic);

TEST_SETUP(ABMAsyncLogic) {
    // Called before each test
}

TEST_TEAR_DOWN(ABMAsyncLogic) {
    // Called after each test
}

// =========================================================================================
// Initialize State Tests
// =========================================================================================

TEST(ABMAsyncLogic, abm_matrix_initialized) {
    Driver driver(nullptr, 0x50, 0xFF, 0xFF);
    TEST_ASSERT_NOT_NULL(&driver._abm_matrix);
}

TEST(ABMAsyncLogic, abm_enqueued_starts_zero) {
    Driver driver(nullptr, 0x50, 0xFF, 0xFF);
    TEST_ASSERT_EQUAL_UINT16(0, driver._abmEnqueued);
}

TEST(ABMAsyncLogic, abm_locked_starts_false) {
    Driver driver(nullptr, 0x50, 0xFF, 0xFF);
    TEST_ASSERT_FALSE(driver._abmLocked);
}

// =========================================================================================
// ABMConfig Enum Model Tests
// =========================================================================================

TEST(ABMAsyncLogic, abm_config_enum_values_match_datasheet_encoding) {
    TEST_ASSERT_EQUAL_UINT8(0xA0, static_cast<uint8_t>(T1_6720MS));
    TEST_ASSERT_EQUAL_UINT8(0x06, static_cast<uint8_t>(T2_840MS));
    TEST_ASSERT_EQUAL_UINT8(0xC0, static_cast<uint8_t>(T3_13440MS));
    TEST_ASSERT_EQUAL_UINT8(0x0A, static_cast<uint8_t>(T4_3360MS));
    TEST_ASSERT_EQUAL_UINT8(0x30, static_cast<uint8_t>(LOOP_BEGIN_T4));
    TEST_ASSERT_EQUAL_UINT8(0x40, static_cast<uint8_t>(LOOP_END_T1));
}

TEST(ABMAsyncLogic, abm_config_struct_stores_fields) {
    ABMConfig cfg = {
        T1_420MS,
        T2_210MS,
        T3_840MS,
        T4_6720MS,
        LOOP_BEGIN_T2,
        LOOP_END_T3,
        0x03A5,
    };

    TEST_ASSERT_EQUAL_UINT8(T1_420MS, cfg.T1);
    TEST_ASSERT_EQUAL_UINT8(T2_210MS, cfg.T2);
    TEST_ASSERT_EQUAL_UINT8(T3_840MS, cfg.T3);
    TEST_ASSERT_EQUAL_UINT8(T4_6720MS, cfg.T4);
    TEST_ASSERT_EQUAL_UINT8(LOOP_BEGIN_T2, cfg.Tbegin);
    TEST_ASSERT_EQUAL_UINT8(LOOP_END_T3, cfg.Tend);
    TEST_ASSERT_EQUAL_UINT16(0x03A5, cfg.Times);
}

// =========================================================================================
// Callback Storage Tests
// =========================================================================================

TEST(ABMAsyncLogic, abm_callbacks_start_empty) {
    Driver driver(nullptr, 0x50, 0xFF, 0xFF);
    TEST_ASSERT_FALSE(driver._abmCallbacks[0]);
    TEST_ASSERT_FALSE(driver._abmCallbacks[1]);
    TEST_ASSERT_FALSE(driver._abmCallbacks[2]);
}

TEST(ABMAsyncLogic, set_abm_callback_stores_callback) {
    Driver driver(nullptr, 0x50, 0xFF, 0xFF);
    auto noop = []() {};

    driver.SetABMCallback(1, noop);
    TEST_ASSERT_TRUE((bool)driver._abmCallbacks[0]);
}

TEST(ABMAsyncLogic, set_abm_callback_abm2) {
    Driver driver(nullptr, 0x50, 0xFF, 0xFF);
    auto noop = []() {};

    driver.SetABMCallback(2, noop);
    TEST_ASSERT_TRUE((bool)driver._abmCallbacks[1]);
}

TEST(ABMAsyncLogic, set_abm_callback_abm3) {
    Driver driver(nullptr, 0x50, 0xFF, 0xFF);
    auto noop = []() {};

    driver.SetABMCallback(3, noop);
    TEST_ASSERT_TRUE((bool)driver._abmCallbacks[2]);
}

TEST(ABMAsyncLogic, set_abm_callback_invokes_directly) {
    Driver driver(nullptr, 0x50, 0xFF, 0xFF);

    int invocation_count = 0;
    driver.SetABMCallback(1, [&invocation_count]() { ++invocation_count; });

    driver._abmCallbacks[0]();
    TEST_ASSERT_EQUAL_INT(1, invocation_count);

    driver._abmCallbacks[0]();
    TEST_ASSERT_EQUAL_INT(2, invocation_count);
}

TEST_GROUP_RUNNER(ABMAsyncLogic) {
    RUN_TEST_CASE(ABMAsyncLogic, abm_matrix_initialized);
    RUN_TEST_CASE(ABMAsyncLogic, abm_enqueued_starts_zero);
    RUN_TEST_CASE(ABMAsyncLogic, abm_locked_starts_false);

    RUN_TEST_CASE(ABMAsyncLogic, abm_config_enum_values_match_datasheet_encoding);
    RUN_TEST_CASE(ABMAsyncLogic, abm_config_struct_stores_fields);

    RUN_TEST_CASE(ABMAsyncLogic, abm_callbacks_start_empty);
    RUN_TEST_CASE(ABMAsyncLogic, set_abm_callback_stores_callback);
    RUN_TEST_CASE(ABMAsyncLogic, set_abm_callback_abm2);
    RUN_TEST_CASE(ABMAsyncLogic, set_abm_callback_abm3);
    RUN_TEST_CASE(ABMAsyncLogic, set_abm_callback_invokes_directly);
}
