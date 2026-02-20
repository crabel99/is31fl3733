/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Cal Abel. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *
 *  Based on the original IS31FL3733 C++ library by Neil Enns
 *  (https://github.com/neilenns/is31fl3733).
 *  This is a nearly complete rewrite for asynchronous DMA-driven operation on
 *  Arduino SAMD.
 *
 *  Asynchronous DMA-driven IS31FL3733 driver implementation.
 *
 *  See is31fl3733_async.hpp and /docs/ASYNC_DMA_ARCHITECTURE.md for details.
 *--------------------------------------------------------------------------------------------*/

#include "is31fl3733.hpp"

#include <utility>

namespace IS31FL3733 {

// =========================================================================================
// Static Members
// =========================================================================================

IS31FL3733 *IS31FL3733::_instance = nullptr;

// =========================================================================================
// Constructor
// =========================================================================================

IS31FL3733::IS31FL3733(TwoWire *wire, uint8_t addr, uint8_t sdbPin, uint8_t irqPin)
    : _hw(wire ? wire->getSercom() : nullptr), _addr(addr), _sdbPin(sdbPin), _irqPin(irqPin),
      _currentPage(0xFF), _pwmEnqueued(0), _pwmLocked(false), _abmEnqueued(0), _abmLocked(false),
      _cmdReturn(0), _cmdError(0), _syncComplete(false), _syncStatus(0), _syncTargetCmd(-1),
      _begun(false), _lastISR(0), _crValue(CR_SSD), _colorOrder(ColorOrder::GRB) {

    // Pre-stage unlock transaction and page buffers
    _crwlTx[0] = PSWL;
    _crwlTx[1] = PSWL_ENABLE;
    _pgSelTx[0] = PSR;
    // _pgSelTx[1] will be set dynamically based on target register page during transactions

    // CRWL Txn (unlock)
    _cmdTxn[0].config = I2C_CFG_STOP;
    _cmdTxn[0].address = _addr;
    _cmdTxn[0].txPtr = _crwlTx;
    _cmdTxn[0].length = 2;
    _cmdCtx[0] = {this, 0, nullptr, nullptr};
    _cmdTxn[0].onComplete = _cmdCallback;
    _cmdTxn[0].user = &_cmdCtx[0];

    // Page Selection Txn
    _cmdTxn[1].config = I2C_CFG_STOP;
    _cmdTxn[1].address = _addr;
    _cmdTxn[1].txPtr = _pgSelTx;
    _cmdTxn[1].length = 2;
    _cmdCtx[1] = {this, 1, nullptr, nullptr};
    _cmdTxn[1].onComplete = _cmdCallback;
    _cmdTxn[1].user = &_cmdCtx[1];

    // Command Write Txn
    _cmdTxn[2].config = I2C_CFG_STOP;
    _cmdTxn[2].address = _addr;
    _cmdTxn[2].txPtr = _cmdTx;
    _cmdCtx[2] = {this, 2, nullptr, nullptr};
    _cmdTxn[2].onComplete = _cmdCallback;
    _cmdTxn[2].user = &_cmdCtx[2];

    // Command Read Txn
    _cmdTxn[3].config = I2C_CFG_READ | I2C_CFG_STOP;
    _cmdTxn[3].address = _addr;
    _cmdTxn[3].rxPtr = _cmdRx;
    _cmdCtx[3] = {this, 3, nullptr, nullptr};
    _cmdTxn[3].onComplete = _cmdCallback;
    _cmdTxn[3].user = &_cmdCtx[3];

    // PWM Txn
    _pwmTxn.config = I2C_CFG_STOP;
    _pwmTxn.address = _addr;
    _pwmTxn.length = 17;
    _pwmTxn.txPtr = nullptr;
    _pwmTxn.onComplete = _txnCallback;
    _pwmTxn.user = this;

    // ABM Txn
    _abmTxn.config = I2C_CFG_STOP;
    _abmTxn.address = _addr;
    _abmTxn.length = 17;
    _abmTxn.txPtr = nullptr;
    _abmTxn.onComplete = _txnModeCallback;
    _abmTxn.user = this;

    // set all LED's on by default (will be updated in begin()) after OSD reads if enabled
    memset(_ledOn, 0xFF, sizeof(_ledOn));

    // Initialize PWM matrix: byte 0 = Page 1 row address (0x00..0x0B)
    for (uint8_t row = 0; row < kHardwareRows; row++)
        _pwm_matrix[row][0] = row; // Page 1 Row Address Register

    // Initialize ABM matrix: byte 0 = Page 2 row address (0x00..0x0B)
    for (uint8_t row = 0; row < kHardwareRows; row++)
        _abm_matrix[row][0] = row; // Page 2 Row Address Register
}

IS31FL3733::~IS31FL3733() {
    if (_begun)
        end();
}

// =========================================================================================
// Initialization (Blocking)
// =========================================================================================

bool IS31FL3733::begin(uint8_t pfs, uint8_t pur, uint8_t pdr) {
    // ---------------------------------------------------------------------------------
    // STEP 1: Configure pins
    // ---------------------------------------------------------------------------------
    // SDB pin (active high)
    if (_sdbPin != 0xFF) {
        pinMode(_sdbPin, OUTPUT);
        digitalWrite(_sdbPin, LOW); // Start disabled
        delay(1);
        digitalWrite(_sdbPin, HIGH); // Enable device
        delay(1);
    }

    // IRQ pin (if provided)
    if (_irqPin != 0xFF) {
        pinMode(_irqPin, INPUT);
        _instance = this; // Set static instance for ISR access
        attachInterrupt(digitalPinToInterrupt(_irqPin), _irqCallback, FALLING);
    }

    // ---------------------------------------------------------------------------------
    // Software RESET
    // ---------------------------------------------------------------------------------

    // Read RESET register to trigger software reset (puts all registers in known state)
    uint8_t dummy;
    if (!_syncRead(RESET, &dummy, 1))
        return false; // RESET read failed

    // ---------------------------------------------------------------------------------
    // Configure Page 3 registers
    // ---------------------------------------------------------------------------------

    // CR: Normal operation (without OSD initially)
    _crValue = static_cast<uint8_t>(CR_SSD | CR_PFS(pfs & 0x03));
    if (!_syncWrite(CR, &_crValue, 1))
        return false; // CR write failed

    // PUR/PDR: Set pull-up/down for de-ghosting (default to all enabled)
    uint8_t purValue = pur & 0b111; // Mask to 3 bits
    uint8_t pdrValue = pdr & 0b111; // Mask to 3 bits
    if (!_syncWrite(SWPUR, &purValue, 1))
        return false; // SWPUR write failed
    if (!_syncWrite(CSPDR, &pdrValue, 1))
        return false; // CSPDR write failed

    // ---------------------------------------------------------------------------------
    // Configure Page 0: LED On/Off and Open/Short Detection
    // ---------------------------------------------------------------------------------

    // Minimal OSD sequence (if IRQ pin provided)
    if (_irqPin != 0xFF) {
        // Step 1: Enable all LEDs in LEDONOFF
        if (!_syncWrite(LEDONOFF, _ledOn, 24))
            return false; // LEDONOFF write failed

        // Step 2: Set GCC to 0x01 for OSD
        uint8_t gcc_osd = 0x01;
        if (!_syncWrite(GCC, &gcc_osd, 1))
            return false; // GCC write failed

        // Step 3: Trigger OSD strobe (CR with OSD bit set)
        uint8_t cr_osd = static_cast<uint8_t>(_crValue | CR_OSD);
        if (!_syncWrite(CR, &cr_osd, 1))
            return false; // CR OSD trigger failed

        // Step 4: Wait for OSD to complete
        delay(10); // 10ms to ensure completion

        // Step 5: Read LEDOPEN and LEDSHORT registers
        if (!_syncRead(LEDOPEN, _ledOpen, 24))
            return false; // LEDOPEN read failed
        if (!_syncRead(LEDSHORT, _ledShort, 24))
            return false; // LEDSHORT read failed

        // Store ISR for debug (read after OSD)
        uint8_t isr_status = 0;
        if (_syncRead((uint16_t)ISR << 8, &isr_status, 1))
            _lastISR = isr_status;

        // Compute LED On/Off mask based on open/short status (turn off faulty LEDs)
        for (size_t i = 0; i < 24; i++)
            _ledOn[i] = _ledOn[i] & ~(_ledOpen[i] | _ledShort[i]);

        // Write updated LEDONOFF mask (if faults detected)
        if (!_syncWrite(LEDONOFF, _ledOn, 24))
            return false; // LEDONOFF update failed

        // Step 6: Restore GCC to normal operating value (0xFF)
        uint8_t gcc_normal = 0xFF;
        if (!_syncWrite(GCC, &gcc_normal, 1))
            return false; // GCC restore failed

        // Step 7: Clear CR OSD bit for normal operation
        if (!_syncWrite(CR, &_crValue, 1))
            return false; // CR clear OSD failed

        // Step 8: Unmask interrupts for runtime fault detection
        uint8_t imr_value = IMR_IO | IMR_IS;
        if (!_syncWrite((uint16_t)IMR << 8, &imr_value, 1))
            return false; // IMR write failed
    } else {
        // No IRQ - just write LEDONOFF and set GCC to normal
        if (!_syncWrite(LEDONOFF, _ledOn, 24))
            return false; // LEDONOFF write failed

        uint8_t gcc_normal = 0xFF;
        if (!_syncWrite(GCC, &gcc_normal, 1))
            return false; // GCC write failed
    }

    // ---------------------------------------------------------------------------------
    // Clear command transaction pointers and set default page to Page 1 (PWM)
    // ---------------------------------------------------------------------------------

    _cmdCtx[2].userCallback = nullptr;
    _cmdCtx[2].user = nullptr;
    _cmdCtx[3].userCallback = nullptr;
    _cmdCtx[3].user = nullptr;

    _ensurePage(1);

    // ---------------------------------------------------------------------------------
    // Initialize PWM to zero for startup (async)
    // ---------------------------------------------------------------------------------

    Fill(0);

    _begun = true;

    return true;
}

void IS31FL3733::end() {
    if (!_begun)
        return;

    _syncRead(CR, _cmdTx, 1); // RESET the device state

    DeviceOff();

    // Detach IRQ if configured
    if (_irqPin != 0xFF) {
        detachInterrupt(digitalPinToInterrupt(_irqPin));
        _instance = nullptr;
    }

    _begun = false;
}

// =========================================================================================
// Device Control
// =========================================================================================

void IS31FL3733::DeviceOn() {
    if (_sdbPin != 0xFF)
        digitalWrite(_sdbPin, HIGH);

    // Write cached CR register (SSD + runtime config bits)
    _asyncWrite(CR, &_crValue, 1, nullptr, nullptr);

    _resumeDataTransfers();
}

void IS31FL3733::DeviceOff() {
    const uint8_t cr_off = 0x00;
    _syncWrite(CR, &cr_off, 1); // Ensure CR write completes before cutting power

    if (_sdbPin != 0xFF)
        digitalWrite(_sdbPin, LOW);
}

void IS31FL3733::SetGCC(uint8_t gcc) {
    _asyncWrite(GCC, &gcc, 1, nullptr, nullptr);
}

void IS31FL3733::SetSWPUR(uint8_t pur) {
    uint8_t purValue = static_cast<uint8_t>(pur & 0x07);
    _asyncWrite(SWPUR, &purValue, 1, nullptr, nullptr);
}

void IS31FL3733::SetCSPDR(uint8_t pdr) {
    uint8_t pdrValue = static_cast<uint8_t>(pdr & 0x07);
    _asyncWrite(CSPDR, &pdrValue, 1, nullptr, nullptr);
}

void IS31FL3733::SetPFS(uint8_t pfs) {
    _crValue = static_cast<uint8_t>((_crValue & ~0x60u) | CR_PFS(pfs & 0x03));
    _asyncWrite(CR, &_crValue, 1, nullptr, nullptr);
}

void IS31FL3733::SetIMR(uint8_t imrMask) {
    uint8_t mask = static_cast<uint8_t>(imrMask & 0x0F);
    _asyncWrite((uint16_t)IMR << 8, &mask, 1, nullptr, nullptr);
}

// =========================================================================================
// PWM Control (Raw Hardware Interface)
// =========================================================================================

void IS31FL3733::SetPixelPWM(uint8_t row, uint8_t col, uint8_t pwm) {
    // guard against out-of-bounds and 0 (1-based indexing)
    if (row > kHardwareRows || col > kHardwareCols || !row || !col)
        return;

    // Convert 1-based to 0-based index
    uint8_t idx = row - 1;

    // [idx][0] is reserved for row address, so column data starts at offset 1
    _pwm_matrix[idx][col] = pwm;

    // Enqueue row for transmission (if not already enqueued)
    uint16_t rowBit = 1 << idx;
    if (_pwmEnqueued & rowBit)
        return; // Already enqueued

    _pwmPendingRows.store_char(idx);
    _pwmEnqueued |= rowBit;

    // Kick transmission if not already in-flight
    if (!_pwmTxn.txPtr && !_pwmLocked)
        _sendRowPWM();
}

void IS31FL3733::SetRowPWM(uint8_t row, const uint8_t *pwmValues) {
    // guard against out-of-bounds and 0 (1-based indexing)
    if (row > kHardwareRows || !row)
        return;

    // Convert 1-based to 0-based index
    uint8_t idx = (row - 1) & 0x0F;

    // Update pwm buffer for the row (starting at offset 1 since [idx][0] is reserved for row
    // address)
    memcpy(_pwm_matrix[idx] + 1, pwmValues, kHardwareCols);

    // Enqueue row for transmission (if not already enqueued)
    uint16_t rowBit = 1 << idx;
    if (_pwmEnqueued & rowBit)
        return; // Already enqueued

    _pwmPendingRows.store_char(idx);
    _pwmEnqueued |= rowBit;

    // Kick transmission if not already in-flight
    if (!_pwmTxn.txPtr && !_pwmLocked)
        _sendRowPWM();
}

// =========================================================================================
// RGB Pixel Control (Logical Coordinates with Color Order)
// =========================================================================================

void IS31FL3733::SetPixelColor(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b) {
    if (row > kLogicalRows || col > kHardwareCols || !row || !col)
        return;

    // Convert 1-based to 0-based index
    uint8_t idx = row - 1;

    // Map logical row to hardware rows based on color order
    // baseRow will be 0, 3, 6, or 9 for logical rows 0-3
    // Add 1 to convert to 1-based indexing for SetPixelPWM
    uint8_t baseRow = idx * 3 + 1;

    // Set PWM for each channel based on color order
    switch (_colorOrder) {
    case ColorOrder::RGB:
        SetPixelPWM(baseRow + 0, col, r); // R on first row
        SetPixelPWM(baseRow + 1, col, g); // G on second row
        SetPixelPWM(baseRow + 2, col, b); // B on third row
        break;
    case ColorOrder::GRB:
        SetPixelPWM(baseRow + 0, col, g); // G on first row
        SetPixelPWM(baseRow + 1, col, r); // R on second row
        SetPixelPWM(baseRow + 2, col, b); // B on third row
        break;
    case ColorOrder::BRG:
        SetPixelPWM(baseRow + 0, col, b); // B on first row
        SetPixelPWM(baseRow + 1, col, r); // R on second row
        SetPixelPWM(baseRow + 2, col, g); // G on third row
        break;
    case ColorOrder::RBG:
        SetPixelPWM(baseRow + 0, col, r); // R on first row
        SetPixelPWM(baseRow + 1, col, b); // B on second row
        SetPixelPWM(baseRow + 2, col, g); // G on third row
        break;
    case ColorOrder::GBR:
        SetPixelPWM(baseRow + 0, col, g); // G on first row
        SetPixelPWM(baseRow + 1, col, b); // B on second row
        SetPixelPWM(baseRow + 2, col, r); // R on third row
        break;
    case ColorOrder::BGR:
        SetPixelPWM(baseRow + 0, col, b); // B on first row
        SetPixelPWM(baseRow + 1, col, g); // G on second row
        SetPixelPWM(baseRow + 2, col, r); // R on third row
        break;
    }
}

// =========================================================================================
// Bulk Operations
// =========================================================================================

void IS31FL3733::Fill(uint8_t pwm) {
    // Fill all rows with the same PWM value
    for (uint8_t row = 0; row < kHardwareRows; row++) {
        memset(_pwm_matrix[row] + 1, pwm, kHardwareCols);

        // Enqueue row
        uint16_t rowBit = 1 << row;
        if (!(_pwmEnqueued & rowBit)) {
            _pwmPendingRows.store_char(row);
            _pwmEnqueued |= rowBit;
        }
    }

    // Kick transmission if not already in-flight
    if (!_pwmTxn.txPtr && !_pwmLocked) {
        _sendRowPWM();
    }
}

// =========================================================================================
// Mode Control (ABM / LED Mode Selection - Page 2)
// =========================================================================================

void IS31FL3733::SetPixelMode(uint8_t row, uint8_t col, ABMMode mode) {
    // guard against out-of-bounds and 0 (1-based indexing)
    if (row > kHardwareRows || col > kHardwareCols || !row || !col)
        return;

    // Convert 1-based to 0-based index
    uint8_t idx = row - 1;

    // [idx][0] is reserved for row address, so column data starts at offset 1
    _abm_matrix[idx][col] = static_cast<uint8_t>(mode);

    // Enqueue row for transmission (if not already enqueued)
    uint16_t rowBit = 1 << idx;
    if (_abmEnqueued & rowBit)
        return; // Already enqueued

    _abmPendingRows.store_char(idx);
    _abmEnqueued |= rowBit;

    // Kick transmission if not already in-flight
    if (!_abmTxn.txPtr && !_abmLocked)
        _sendRowMode();
}

void IS31FL3733::SetRowMode(uint8_t row, ABMMode mode) {
    // guard against out-of-bounds and 0 (1-based indexing)
    if (row > kHardwareRows || !row)
        return;

    // Convert 1-based to 0-based index
    uint8_t idx = (row - 1) & 0x0F;

    // Update mode buffer for the row (starting at offset 1 since [idx][0] is reserved for row
    // address)
    memset(_abm_matrix[idx] + 1, static_cast<uint8_t>(mode), kHardwareCols);

    // Enqueue row for transmission (if not already enqueued)
    uint16_t rowBit = 1 << idx;
    if (_abmEnqueued & rowBit)
        return; // Already enqueued

    _abmPendingRows.store_char(idx);
    _abmEnqueued |= rowBit;

    // Kick transmission if not already in-flight
    if (!_abmTxn.txPtr && !_abmLocked)
        _sendRowMode();
}

void IS31FL3733::SetMatrixMode(ABMMode mode) {
    // Fill all rows with the same mode value (Page 2)
    for (uint8_t row = 0; row < kHardwareRows; row++) {
        memset(_abm_matrix[row] + 1, static_cast<uint8_t>(mode), kHardwareCols);

        // Enqueue row
        uint16_t rowBit = 1 << row;
        if (!(_abmEnqueued & rowBit)) {
            _abmPendingRows.store_char(row);
            _abmEnqueued |= rowBit;
        }
    }

    // Kick transmission if not already in-flight
    if (!_abmTxn.txPtr && !_abmLocked) {
        _sendRowMode();
    }
}

ABMMode IS31FL3733::GetPixelMode(uint8_t row, uint8_t col) const {
    // guard against out-of-bounds and 0 (1-based indexing)
    if (row > kHardwareRows || col > kHardwareCols || !row || !col)
        return ABMMode::PWM_MODE;

    // Convert 1-based to 0-based index
    uint8_t idx = row - 1;

    // Read from cached ABM matrix
    return static_cast<ABMMode>(_abm_matrix[idx][col]);
}

void IS31FL3733::SetPixelColorMode(uint8_t row, uint8_t col, ABMMode mode) {
    if (row > kLogicalRows || col > kHardwareCols || !row || !col)
        return;

    // Convert 1-based to 0-based index
    uint8_t idx = row - 1;

    // Map logical row to hardware rows based on color order
    // baseRow will be 0, 3, 6, or 9 for logical rows 0-3
    // Add 1 to convert to 1-based indexing for SetPixelMode
    uint8_t baseRow = idx * 3 + 1;

    // Set mode for each channel (all three use the same ABM mode for "color")
    SetPixelMode(baseRow + 0, col, mode);
    SetPixelMode(baseRow + 1, col, mode);
    SetPixelMode(baseRow + 2, col, mode);
}

// =========================================================================================
// Core Transaction Methods
// =========================================================================================

void IS31FL3733::_ensurePage(uint8_t page) {
    // Skip page select if we're already on the target page
    if (_currentPage == page)
        return;

    // Enqueue unlock transaction (pre-staged in constructor)
    _hw->enqueueWIRE(&_cmdTxn[0]);

    // Update and enqueue page select transaction
    _pgSelTx[0] = PSR;
    _pgSelTx[1] = page & 0b11; // Mask to 2 bits (0-3)
    _hw->enqueueWIRE(&_cmdTxn[1]);

    // Update tracked page
    _currentPage = page;
}

void IS31FL3733::_resumeDataTransfers() {
    _pwmLocked = false; // Clear PWM lock flag
    _abmLocked = false; // Clear ABM lock flag
    _sendRowPWM();      // Resume pending PWM writes
    _sendRowMode();     // Resume pending ABM writes
}

bool IS31FL3733::_syncWrite(uint16_t pagereg, const uint8_t *data, uint8_t len) {
    _syncComplete = false;
    _syncStatus = 0;
    _syncTargetCmd = 2;
    _cmdError &= ~((1u << 0) | (1u << 1) | (1u << 2));
    _cmdCtx[2].initialStatus = 0;

    _asyncWrite(pagereg, data, len, nullptr, nullptr);

    const unsigned long start = millis();
    while (!_syncComplete && (millis() - start < 100ul))
        ; // Spin until complete or timeout

    if (!_syncComplete) {
        _syncStatus = -1;
        _syncTargetCmd = -1;
        _syncComplete = true;
        return false;
    }

    return (_syncStatus == 0) && ((_cmdError & ((1u << 0) | (1u << 1) | (1u << 2))) == 0) &&
           (_cmdCtx[2].initialStatus == 0);
}

bool IS31FL3733::_syncRead(uint16_t pagereg, uint8_t *dest, uint8_t len) {
    _syncComplete = false;
    _syncStatus = 0;
    _syncTargetCmd = 3;
    _cmdError &= ~((1u << 0) | (1u << 1) | (1u << 2) | (1u << 3));
    _cmdCtx[3].initialStatus = 0;

    _asyncRead(pagereg, dest, len, nullptr, nullptr);

    const unsigned long start = millis();
    while (!_syncComplete && (millis() - start < 100ul))
        ; // Spin until complete or timeout

    if (!_syncComplete) {
        _syncStatus = -1;
        _syncTargetCmd = -1;
        _syncComplete = true;
        return false;
    }

    return (_syncStatus == 0) &&
           ((_cmdError & ((1u << 0) | (1u << 1) | (1u << 2) | (1u << 3))) == 0) &&
           (_cmdCtx[3].initialStatus == 0);
}

// =========================================================================================
// ABM Configuration (Page 3 Register Updates)
// =========================================================================================

void IS31FL3733::SetABMCallback(uint8_t abmNum, std::function<void()> callback) {
    if (abmNum < 1 || abmNum > 3)
        return;

    _abmCallbacks[abmNum - 1] = callback;
}

void IS31FL3733::ConfigureABM(uint8_t abmNumber, const ABMConfig &config) {
    if (abmNumber < 1 || abmNumber > 3)
        return;
    uint16_t pagereg = 0;
    // Route to specific ABM configurator
    switch (abmNumber) {
    case 1:
        pagereg = ABM1CR;
        break;
    case 2:
        pagereg = ABM2CR;
        break;
    case 3:
        pagereg = ABM3CR;
        break;
    }

    const uint8_t cfg[4] = {
        static_cast<uint8_t>(static_cast<uint8_t>(config.T1) | static_cast<uint8_t>(config.T2)),
        static_cast<uint8_t>(static_cast<uint8_t>(config.T3) | static_cast<uint8_t>(config.T4)),
        static_cast<uint8_t>(static_cast<uint8_t>(config.Tend) |
                             static_cast<uint8_t>(config.Tbegin) |
                             static_cast<uint8_t>((config.Times >> 8) & 0x0F)),
        static_cast<uint8_t>(config.Times & 0xFF),
    };
    _asyncWrite(pagereg, cfg, 4, nullptr, nullptr);

    // Write 1 byte of zeros to TUR (0x0E) to complete latch (per datasheet: "Any write to
    // 0Eh will latch the ABM configuration")
    uint8_t turPadding[1] = {0};
    _asyncWrite(TUR, turPadding, 1, nullptr, nullptr);
}

void IS31FL3733::ConfigureABM1(const ABMConfig &config) {
    ConfigureABM(1, config);
}

void IS31FL3733::ConfigureABM2(const ABMConfig &config) {
    ConfigureABM(2, config);
}

void IS31FL3733::ConfigureABM3(const ABMConfig &config) {
    ConfigureABM(3, config);
}

void IS31FL3733::EnableABM(bool enable) {
    if (enable)
        _crValue |= CR_BEN;
    else
        _crValue &= static_cast<uint8_t>(~CR_BEN);

    _asyncWrite(CR, &_crValue, 1, nullptr, nullptr);
}

void IS31FL3733::TriggerABM() {
    // Ensure Page 3
    _ensurePage(3);

    // Write any value to TUR (0x0E) to latch configuration
    // Per datasheet: "Any write to 0Eh will latch the ABM configuration"
    // We write 0x00 as the trigger value
    uint8_t tur_value = 0x00;
    _asyncWrite(TUR, &tur_value, 1, nullptr, nullptr);
}

// =========================================================================================
// Static Callbacks
// =========================================================================================

void IS31FL3733::_osbCallback(void *user, int status) {
    IS31FL3733 *self = (IS31FL3733 *)user;

    // Update LED On/Off mask: disable faulty LEDs
    for (size_t i = 0; i < 24; i++)
        self->_ledOn[i] = self->_ledOn[i] & ~(self->_ledOpen[i] | self->_ledShort[i]);

    // Write updated LED On/Off register
    self->_asyncWrite(LEDONOFF, self->_ledOn, 24, nullptr, nullptr);

    // Restore Page 1 and resume PWM/ABM
    self->_resumeDataTransfers();
}

void IS31FL3733::_cmdCallback(void *user, int status) {
    auto *context = static_cast<CmdTxnContext *>(user);
    if (!context || !context->self)
        return;

    IS31FL3733 *self = context->self;
    const uint8_t bit = static_cast<uint8_t>(1u << context->index);

    self->_cmdReturn |= bit;
    if (status != 0)
        self->_cmdError |= bit;

    if (self->_syncTargetCmd == static_cast<int8_t>(context->index)) {
        self->_syncStatus = status;
        self->_syncComplete = true;
        self->_syncTargetCmd = -1;
    }

    // For non-final phases: accumulate status in next phase's context
    if (!context->isFinal && context->index < 3) {
        // Chain accumulated status forward: next phase sees all previous failures
        self->_cmdCtx[context->index + 1].initialStatus |= status;
        return; // Don't invoke user callback on intermediate phase
    }

    // Only invoke user callback on final transaction
    if (context->isFinal && context->userCallback)
        context->userCallback(context->user, status);
}

// =========================================================================================
// ABM Callback Wrappers (Static Entry Points for PendSV Dispatch)
// =========================================================================================

void IS31FL3733::_abm1CallbackWrapper() {
    if (_instance && _instance->_abmCallbacks[0])
        _instance->_abmCallbacks[0]();
}

void IS31FL3733::_abm2CallbackWrapper() {
    if (_instance && _instance->_abmCallbacks[1])
        _instance->_abmCallbacks[1]();
}

void IS31FL3733::_abm3CallbackWrapper() {
    if (_instance && _instance->_abmCallbacks[2])
        _instance->_abmCallbacks[2]();
}

} // namespace IS31FL3733
