/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Cal Abel. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *
 *  Based on the original IS31FL3733 C++ library by Neil Enns
 *  (https://github.com/neilenns/is31fl3733).
 *  This is a nearly complete rewrite for asynchronous DMA-driven operation on
 *  Arduino SAMD.
 *
 *  Asynchronous DMA-driven IS31FL3733 driver for Arduino SAMD (SimIO framework).
 *
 *  Architecture:
 *  - Non-blocking I2C via SERCOM transaction descriptors with DMA
 *  - Lock-free ring buffer for PWM row updates (12 rows, bitfield-tracked)
 *  - Single in-flight PWM transaction (_pwmTxn) with shadow buffer (_pwmTxPtr[17])
 *  - Separate command transaction chain (_cmdTxn[4]) for page select / fault handling
 *  - Default Page 1 (PWM) - no page switching for normal writes
 *  - IRQ-driven fault detection (open/short) with async transaction chain
 *  - Color order support (RGB, GRB, BRG, RBG, GBR, BGR)
 *
 *  See /docs/ASYNC_DMA_ARCHITECTURE.md for full design details.
 *--------------------------------------------------------------------------------------------*/
#pragma once

#include <Arduino.h>
#include <Wire.h> // TwoWire I2C interface
#include <functional>

namespace IS31FL3733 {
// -----------------------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------------------

constexpr uint8_t kHardwareRows = 12; ///< IS31FL3733 SW lines (hardware rows)
constexpr uint8_t kHardwareCols = 16; ///< IS31FL3733 CS lines (hardware columns)
constexpr uint8_t kLedCount = kHardwareRows * kHardwareCols; ///< Total LED count (192)

constexpr uint8_t kLogicalRows = 4; ///< RGB pixel rows (3 channels x 4 rows = 12)

// -----------------------------------------------------------------------------------------
// Register Addresses (Common + Paged)
// -----------------------------------------------------------------------------------------

/// @brief Common registers accessible from any page.
///
/// These registers are in the common register space and can be accessed
/// regardless of the current page selection.
enum CommonRegister : uint8_t {
    PSR = 0xFD,  ///< Page Select Register (write only)
    PSWL = 0xFE, ///< Page Select Write Lock (read/write)
    IMR = 0xF0,  ///< Interrupt Mask Register (write only)
    ISR = 0xF1,  ///< Interrupt Status Register (read only)
};

/// @brief Paged registers accessible after page selection via PSR.
///
/// These registers are accessed by first writing the page number to PSR,
/// then reading/writing at the specified offset. The upper byte encodes
/// the page number (0-3), lower byte is the register offset.
enum PagedRegister : uint16_t {
    LEDONOFF = 0x0000, ///< Page 0: LED On/Off (24 bytes, 0x00..0x17)
    LEDOPEN = 0x0018,  ///< Page 0: LED Open Status (24 bytes, 0x18..0x2F, read only)
    LEDSHORT = 0x0030, ///< Page 0: LED Short Status (24 bytes, 0x30..0x47, read only)
    LEDPWM = 0x0100,   ///< Page 1: PWM Duty Cycle (per LED, write only)
    LEDABM = 0x0200,   ///< Page 2: Auto Breath Mode (per LED, write only)
    CR = 0x0300,       ///< Page 3: Configuration Register (write only)
    GCC = 0x0301,      ///< Page 3: Global Current Control (write only)
    ABM1CR = 0x0302,   ///< Page 3: ABM-1 Control (write only)
    ABM2CR = 0x0306,   ///< Page 3: ABM-2 Control (write only)
    ABM3CR = 0x030A,   ///< Page 3: ABM-3 Control (write only)
    TUR = 0x030E,      ///< Page 3: Time Update Register (write only)
    SWPUR = 0x030F,    ///< Page 3: SWy Pull-Up Resistor (write only)
    CSPDR = 0x0310,    ///< Page 3: CSx Pull-Down Resistor (read only)
    RESET = 0x0311,    ///< Page 3: RESET Register (read only)
};

// -----------------------------------------------------------------------------------------
// Register Configuration Options
// -----------------------------------------------------------------------------------------

/// @brief Page Select Write Lock register values.
///
/// Controls whether the PSR (Page Select Register) can be modified.
/// Write PSWL_ENABLE to lock, PSWL_DISABLE to unlock.
enum PswlOptions : uint8_t {
    PSWL_DISABLE = 0x00,
    PSWL_ENABLE = 0xC5,
};

/// @brief Interrupt Mask Register bit flags.
///
/// Write these flags to IMR to enable/disable specific interrupt sources.
/// Multiple flags can be OR'd together.
enum ImrOptions : uint8_t {
    IMR_IAC = 0x08, ///< Auto Clear Interrupt
    IMR_IAB = 0x04, ///< Auto Breath Interrupt
    IMR_IS = 0x02,  ///< Short Interrupt
    IMR_IO = 0x01,  ///< Open Interrupt
};

/// @brief Interrupt Status Register bit flags.
///
/// Read ISR to determine which interrupt(s) occurred.
/// Bits are automatically cleared when read (if IMR_IAC is set).
enum IsrOptions : uint8_t {
    ISR_ABM3 = 0x10, ///< ABM-3 Finish
    ISR_ABM2 = 0x08, ///< ABM-2 Finish
    ISR_ABM1 = 0x04, ///< ABM-1 Finish
    ISR_SB = 0x02,   ///< Short Bit
    ISR_OB = 0x01,   ///< Open Bit
};

/// @brief Configuration Register (CR) bit flags.
///
/// Controls device operating mode, synchronization, and power state.
/// Multiple flags can be OR'd together.
enum CrOptions : uint8_t {
    CR_SYNC_MASTER = 0x40,
    CR_SYNC_SLAVE = 0x80,
    CR_OSD = 0x04, ///< Open/Short Detection Enable
    CR_BEN = 0x02, ///< Auto Breath Mode Enable
    CR_SSD = 0x01, ///< Software Shutdown (1 = normal, 0 = shutdown)
};

/// @brief Helper function to encode page select bits for CR register.
///
/// @param page Page number (0-3)
/// @return CR register value with PFS bits [6:5] set
inline uint8_t CR_PFS(uint8_t page) {
    return (page & 0x03) << 5;
} // PSR bits are [6:5] for page select

// -----------------------------------------------------------------------------------------
// Color Order Enum
// -----------------------------------------------------------------------------------------

/// @brief RGB channel order for mapping pixel colors to hardware SW lines.
///
/// Defines how each RGB pixel's channels map to consecutive hardware rows.
/// Used by IS31FL3733RgbMatrix for color channel routing.
enum class ColorOrder : uint8_t {
    RGB = 0,
    GRB = 1,
    BRG = 2,
    RBG = 3,
    GBR = 4,
    BGR = 5,
};

/// @brief Auto Breath Mode (ABM) selection.
///
/// Controls which breathing effect mode is active for the LED matrix.
/// PWM_MODE = normal PWM control, ABM1/2/3 = automatic breathing patterns.
enum class ABMMode : uint8_t {
    PWM_MODE = 0x00, // Renamed from PWM to avoid Arduino Due macro conflict
    ABM1 = 0x01,
    ABM2 = 0x02,
    ABM3 = 0x03,
};

/// @brief ABM T1 (fade-in) selection values.
///
/// Encoded for ABMxCR byte 0, bits [7:5].
/// See datasheet ABM timing table for exact visual waveform behavior.
enum AbmT1 : uint8_t {
    T1_210MS = 0x00,   ///< 210 ms fade-in
    T1_420MS = 0x20,   ///< 420 ms fade-in
    T1_840MS = 0x40,   ///< 840 ms fade-in
    T1_1680MS = 0x60,  ///< 1680 ms fade-in
    T1_3360MS = 0x80,  ///< 3360 ms fade-in
    T1_6720MS = 0xA0,  ///< 6720 ms fade-in
    T1_13440MS = 0xC0, ///< 13440 ms fade-in
    T1_26880MS = 0xE0, ///< 26880 ms fade-in
};

/// @brief ABM T2 (hold-high) selection values.
///
/// Encoded for ABMxCR byte 0, bits [4:1].
enum AbmT2 : uint8_t {
    T2_0MS = 0x00,     ///< 0 ms hold-high
    T2_210MS = 0x02,   ///< 210 ms hold-high
    T2_420MS = 0x04,   ///< 420 ms hold-high
    T2_840MS = 0x06,   ///< 840 ms hold-high
    T2_1680MS = 0x08,  ///< 1680 ms hold-high
    T2_3360MS = 0x0A,  ///< 3360 ms hold-high
    T2_6720MS = 0x0C,  ///< 6720 ms hold-high
    T2_13440MS = 0x0E, ///< 13440 ms hold-high
    T2_26880MS = 0x10, ///< 26880 ms hold-high
};

/// @brief ABM T3 (fade-out) selection values.
///
/// Encoded for ABMxCR byte 1, bits [7:5].
enum AbmT3 : uint8_t {
    T3_210MS = 0x00,   ///< 210 ms fade-out
    T3_420MS = 0x20,   ///< 420 ms fade-out
    T3_840MS = 0x40,   ///< 840 ms fade-out
    T3_1680MS = 0x60,  ///< 1680 ms fade-out
    T3_3360MS = 0x80,  ///< 3360 ms fade-out
    T3_6720MS = 0xA0,  ///< 6720 ms fade-out
    T3_13440MS = 0xC0, ///< 13440 ms fade-out
    T3_26880MS = 0xE0, ///< 26880 ms fade-out
};

/// @brief ABM T4 (off-time) selection values.
///
/// Encoded for ABMxCR byte 1, bits [4:1].
enum AbmT4 : uint8_t {
    T4_0MS = 0x00,      ///< 0 ms off-time
    T4_210MS = 0x02,    ///< 210 ms off-time
    T4_420MS = 0x04,    ///< 420 ms off-time
    T4_840MS = 0x06,    ///< 840 ms off-time
    T4_1680MS = 0x08,   ///< 1680 ms off-time
    T4_3360MS = 0x0A,   ///< 3360 ms off-time
    T4_6720MS = 0x0C,   ///< 6720 ms off-time
    T4_13440MS = 0x0E,  ///< 13440 ms off-time
    T4_26880MS = 0x10,  ///< 26880 ms off-time
    T4_53760MS = 0x12,  ///< 53760 ms off-time
    T4_107520MS = 0x14, ///< 107520 ms off-time
};

/// @brief ABM loop-begin selector.
///
/// Encoded for ABMxCR byte 2, bits [5:4].
enum AbmLoopBegin : uint8_t {
    LOOP_BEGIN_T1 = 0x00, ///< Loop restarts from T1 segment
    LOOP_BEGIN_T2 = 0x10, ///< Loop restarts from T2 segment
    LOOP_BEGIN_T3 = 0x20, ///< Loop restarts from T3 segment
    LOOP_BEGIN_T4 = 0x30, ///< Loop restarts from T4 segment
};

/// @brief ABM loop-end selector.
///
/// Encoded for ABMxCR byte 2, bit [6].
enum AbmLoopEnd : uint8_t {
    LOOP_END_T3 = 0x00, ///< Loop terminates at end of T3
    LOOP_END_T1 = 0x40, ///< Loop terminates at end of T1
};

/// @brief Maximum finite loop count that fits ABMxCR[2:3] LTA/LTB fields.
constexpr int kAbmLoopTimesMax = 0x0FFF;
/// @brief Special loop count value for continuous looping.
constexpr int kAbmLoopForever = 0x0000;

/// @brief High-level ABM configuration values before register packing.
///
/// Packed by ConfigureABM() as:
/// - ABMxCR+0 = T1 | T2
/// - ABMxCR+1 = T3 | T4
/// - ABMxCR+2 = Tend | Tbegin | ((Times >> 8) & 0x0F)
/// - ABMxCR+3 = (Times & 0xFF)
struct ABMConfig {
    AbmT1 T1;            ///< Fade-in segment duration selector
    AbmT2 T2;            ///< Hold-high segment duration selector
    AbmT3 T3;            ///< Fade-out segment duration selector
    AbmT4 T4;            ///< Off-time segment duration selector
    AbmLoopBegin Tbegin; ///< Loop begin segment selector
    AbmLoopEnd Tend;     ///< Loop end segment selector
    uint16_t Times;      ///< Loop count (0 = kAbmLoopForever, max = kAbmLoopTimesMax)
};

// -----------------------------------------------------------------------------------------
// IS31FL3733 Async Driver Class
// -----------------------------------------------------------------------------------------

class IS31FL3733 {
  public:
    // ---------------------------------------------------------------------------------------
    // Constructor / Destructor
    // ---------------------------------------------------------------------------------------

    /// @brief Construct a new IS31FL3733 driver instance.
    /// @param wire Pointer to TwoWire I2C interface.
    /// @param addr 7-bit I2C address (typically 0x50).
    /// @param sdbPin Optional SDB (shutdown) pin (active high). Use 0xFF to disable.
    /// @param irqPin Optional IRQ pin for open/short detection. Use 0xFF to disable.
    IS31FL3733(TwoWire *wire, uint8_t addr = 0x50, uint8_t sdbPin = 0xFF, uint8_t irqPin = 0xFF);

    ~IS31FL3733();

    // ---------------------------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------------------------

    /// @brief Initialize the device (blocking for initial config).
    /// @param pfs PWM Frequency Setting value (0..3) for initial configuration. See Table 13 for
    /// setting values. Default is 0 (slowest PWM frequency, 8.4 kHz).
    /// @param pur Pull-up resistor setting for de-ghosting (3 bits, 0..7). Default is
    /// 0b111 (32 kOhm pull-up). See Table 19 for resistor values.
    /// @param pdr Pull-down resistor setting for de-ghosting (3 bits, 0..7). Default is
    /// 0b111 (32 kOhm pull-down). See Table 20 for resistor values.
    /// @return true if successful, false otherwise.
    bool begin(uint8_t pfs = 0, uint8_t pur = 0b111, uint8_t pdr = 0b111);

    /// @brief RESET device and disable (hardware shutdown).
    /// Reads RESET register to trigger software reset, then disables device.
    /// Useful for cleanup and testing.
    void end();

    // ---------------------------------------------------------------------------------------
    // Device Control
    // ---------------------------------------------------------------------------------------

    /// @brief Enable the device (hardware and software startup).
    void DeviceOn();

    /// @brief Disable the device (hardware and software shutdown).
    void DeviceOff();

    // ---------------------------------------------------------------------------------------
    // Runtime Configuration (Page 3/Common)
    // ---------------------------------------------------------------------------------------

    /// @brief Set global current control register (Page 3 GCC).
    /// @param gcc Global current value (0x00..0xFF).
    void SetGCC(uint8_t gcc);

    /// @brief Set SW pull-up resistor control (Page 3 SWPUR).
    /// @param pur Pull-up value (lower 3 bits are used).
    void SetSWPUR(uint8_t pur);

    /// @brief Set CS pull-down resistor control (Page 3 CSPDR).
    /// @param pdr Pull-down value (lower 3 bits are used).
    void SetCSPDR(uint8_t pdr);

    /// @brief Set PWM frequency selection bits in CR (PFS[6:5]).
    /// Preserves other tracked CR bits (e.g., ABM enable).
    /// @param pfs PWM frequency selector (0..3).
    void SetPFS(uint8_t pfs);

    /// @brief Set interrupt mask register (Common IMR).
    /// @param imrMask IMR bitmask (IMR_IO/IMR_IS/IMR_IAB/IMR_IAC).
    void SetIMR(uint8_t imrMask);

    // ---------------------------------------------------------------------------------------
    // PWM & ABM Control (Raw Hardware Interface)
    // ---------------------------------------------------------------------------------------

    /// @brief Set PWM duty cycle for a single LED (hardware coordinates).
    /// @param row Hardware row (1..12 = SW1..SW12).
    /// @param col Hardware column (1..16 = CS1..CS16).
    /// @param pwm PWM value (0..255).
    void SetPixelPWM(uint8_t row, uint8_t col, uint8_t pwm);

    /// @brief Set PWM values for an entire row (hardware coordinates).
    /// @param row Hardware row (1..12 = SW1..SW12).
    /// @param pwmValues Array of 16 PWM values.
    void SetRowPWM(uint8_t row, const uint8_t *pwmValues);

    /// @brief Set mode for a single LED (hardware coordinates, Page 2).
    /// @param row Hardware row (1..12 = SW1..SW12).
    /// @param col Hardware column (1..16 = CS1..CS16).
    /// @param mode LED mode value (PWM/ABM1/ABM2/ABM3).
    void SetPixelMode(uint8_t row, uint8_t col, ABMMode mode);

    /// @brief Set mode for an entire row (hardware coordinates, Page 2).
    /// @param row Hardware row (1..12 = SW1..SW12).
    /// @param mode LED mode value (PWM/ABM1/ABM2/ABM3).
    void SetRowMode(uint8_t row, ABMMode mode);

    /// @brief Get mode for a single LED (hardware coordinates, Page 2).
    /// @param row Hardware row (1..12 = SW1..SW12).
    /// @param col Hardware column (1..16 = CS1..CS16).
    /// @return Current cached LED mode.
    ABMMode GetPixelMode(uint8_t row, uint8_t col) const;

    // ---------------------------------------------------------------------------------------
    // RGB Pixel Control (Logical Coordinates with Color Order)
    // ---------------------------------------------------------------------------------------

    /// @brief Set RGB color for a logical pixel (with color order mapping).
    /// @param row Logical pixel row (1..4).
    /// @param col Logical pixel column (1..16).
    /// @param r Red channel (0..255).
    /// @param g Green channel (0..255).
    /// @param b Blue channel (0..255).
    void SetPixelColor(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b);

    /// @brief Set mode for a logical RGB pixel (with color order mapping).
    /// @param row Logical pixel row (1..4).
    /// @param col Logical pixel column (1..16).
    /// @param mode ABM mode.
    void SetPixelColorMode(uint8_t row, uint8_t col, ABMMode mode);

    /// @brief Set the color order for RGB pixel mapping.
    /// @param order Color order (RGB, GRB, BRG, RBG, GBR, BGR).
    void SetColorOrder(ColorOrder order) {
        _colorOrder = order;
    }

    /// @brief Get the current color order.
    /// @return Current color order.
    ColorOrder GetColorOrder() const {
        return _colorOrder;
    }

    // ---------------------------------------------------------------------------------------
    // Bulk Operations
    // ---------------------------------------------------------------------------------------

    /// @brief Fill the entire matrix with a PWM value.
    /// @param pwm PWM value (0..255).
    void Fill(uint8_t pwm = 0);

    /// @brief Set mode for the entire matrix (Page 2).
    /// @param mode LED mode value (PWM/ABM1/ABM2/ABM3).
    void SetMatrixMode(ABMMode mode);

    /// @brief Register callback for ABM1/2/3 completion.
    /// @param abmNum ABM selector (1..3).
    /// @param callback Invoked when selected ABM completes.
    void SetABMCallback(uint8_t abmNum, std::function<void()> callback);

    // ---------------------------------------------------------------------------------------
    // ABM Page 3 Control
    // ---------------------------------------------------------------------------------------

    /// @brief Configure ABM timing/loop registers for ABM1/2/3 on Page 3.
    /// @param abmNumber ABM selector (1..3).
    /// @param config ABM timing/loop configuration fields.
    void ConfigureABM(uint8_t abmNumber, const ABMConfig &config);

    /// @brief Configure ABM-1 control registers (Page 3 ABM1CR..ABM1CR+4).
    void ConfigureABM1(const ABMConfig &config);

    /// @brief Configure ABM-2 control registers (Page 3 ABM2CR..ABM2CR+4).
    void ConfigureABM2(const ABMConfig &config);

    /// @brief Configure ABM-3 control registers (Page 3 ABM3CR..ABM3CR+4).
    void ConfigureABM3(const ABMConfig &config);

    /// @brief Enable or disable ABM engine in CR (Page 3).
    /// @param enable True to set CR_BEN, false to clear it.
    void EnableABM(bool enable = true);

    /// @brief Latch ABM timing updates by writing TUR on Page 3.
    void TriggerABM();

    // ---------------------------------------------------------------------------------------
    // Fault Detection (Read-Only)
    // ---------------------------------------------------------------------------------------

    /// @brief Get the cached LED open status (from last fault detection).
    /// @return Pointer to 24-byte array (Page 0, 0x18..0x2F).
    const uint8_t *GetLEDOpen() const {
        return _ledOpen;
    }

    /// @brief Get the cached LED short status (from last fault detection).
    /// @return Pointer to 24-byte array (Page 0, 0x30..0x47).
    const uint8_t *GetLEDShort() const {
        return _ledShort;
    }

#ifndef TEST_NATIVE
  private:
#endif // TEST_NATIVE
    // ---------------------------------------------------------------------------------------
    // Hardware Handles
    // ---------------------------------------------------------------------------------------

    SERCOM *_hw;          ///< SERCOM I2C hardware interface
    uint8_t _addr;        ///< 7-bit I2C address
    uint8_t _sdbPin;      ///< SDB (shutdown) pin (0xFF = not used)
    uint8_t _irqPin;      ///< IRQ pin (0xFF = not used)
    uint8_t _currentPage; ///< Tracks current page (0-3, 0xFF=unknown) to minimize page selects

    static IS31FL3733 *_instance; ///< Static instance pointer for ISR access

    // ---------------------------------------------------------------------------------------
    // PWM Matrix and Transaction State (Page 1)
    // ---------------------------------------------------------------------------------------

    uint8_t _pwm_matrix[kHardwareRows]
                       [kHardwareCols + 1]; ///< [row][0] = Page 1 row addr, [1..16] = PWM data
    SercomTxn _pwmTxn;                      ///< Single in-flight PWM transaction descriptor
    uint8_t _pwmTxPtr[kHardwareCols + 1];   ///< Shadow buffer for current transaction

    RingBufferN<kHardwareRows> _pwmPendingRows; ///< Ring buffer of row indices to write
    uint16_t _pwmEnqueued; ///< Bitfield tracking enqueued rows (0x0001..0x0FFF)

    bool _pwmLocked; ///< True when command chain has preempted PWM

    // ---------------------------------------------------------------------------------------
    // Mode Matrix and Transaction State (Page 2)
    // ---------------------------------------------------------------------------------------
    uint8_t _abm_matrix[kHardwareRows]
                       [kHardwareCols + 1]; ///< [row][0] = Page 2 row addr, [1..16] = ABM data
    SercomTxn _abmTxn;                      ///< Single in-flight ABM transaction descriptor
    uint8_t _abmTxPtr[kHardwareCols + 1];   ///< Shadow buffer for current transaction

    RingBufferN<kHardwareRows> _abmPendingRows; ///< Ring buffer of row indices to write
    uint16_t _abmEnqueued; ///< Bitfield tracking enqueued rows (0x0001..0x0FFF)

    bool _abmLocked; ///< True when command chain has preempted ABM

    std::function<void()> _abmCallbacks[3]; ///< Completion callbacks for ABM1/2/3

    // ---------------------------------------------------------------------------------------
    // Command Transaction Chain (Non-PWM Operations)
    // ---------------------------------------------------------------------------------------

    SercomTxn _cmdTxn[4]; ///< [0]=unlock, [1]=page, [2]=write, [3]=read

    struct CmdTxnContext {
        IS31FL3733 *self;
        uint8_t index;
        void (*userCallback)(void *, int);
        void *user;
        bool isFinal;      ///< If true, invoke userCallback on this transaction
        int initialStatus; ///< Initial status (e.g., from first transaction)
    };

    CmdTxnContext _cmdCtx[4]; ///< Context for each command transaction (for callbacks)
    uint8_t _crwlTx[2];       ///< Pre-staged unlock transaction {PSWL, PSWL_ENABLE}
    uint8_t _pgSelTx[2];      ///< Page select transaction buffer {PSR, page}
    uint8_t _cmdTx[25];       ///< TX buffer (max: 1 addr + 24 data for LED On/Off)
    uint8_t _cmdRx[24];       ///< RX buffer (max: 24 bytes for LED Open/Short)

    volatile uint8_t _cmdReturn; ///< Bitfield of completed command transactions (bits 0..3)
    volatile uint8_t _cmdError;  ///< Bitfield of command transactions with nonzero status

    volatile bool _syncComplete;
    volatile int _syncStatus;
    volatile int8_t _syncTargetCmd;

    bool _begun; ///< True after successful begin(); used for destructor auto-end safety.

    // ---------------------------------------------------------------------------------------
    // Fault Detection and ISR Cache
    // ---------------------------------------------------------------------------------------

    uint8_t _ledOpen[24];  ///< Cached LED open status (Page 0, 0x18)
    uint8_t _ledShort[24]; ///< Cached LED short status (Page 0, 0x30)
    uint8_t _ledOn[24];    ///< Computed LED On/Off mask (Page 0, 0x00)
    uint8_t _lastISR;      ///< Cached ISR value from last fault detection

    // ---------------------------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------------------------

    uint8_t _crValue;       ///< Shadow of CR register state for runtime updates
    ColorOrder _colorOrder; ///< RGB pixel color order

    // ---------------------------------------------------------------------------------------
    // Private Methods (Transaction Management)
    // ---------------------------------------------------------------------------------------

    /// @brief Dequeue and transmit next pending PWM row.
    inline void _sendRowPWM();

    /// @brief Dequeue and transmit next pending ABM row.
    inline void _sendRowMode();

    /// @brief Ensure we're on the target page (skips if already there, enqueues unlock +
    /// page-select if needed).
    /// @param page Page number (0..3).
    void _ensurePage(uint8_t page);

    /// @brief Restore Page 1, clear _pwmLocked, resume PWM writes.
    void _resumeDataTransfers();

    /// @brief Generic async write helper (inline for performance).
    inline void _asyncWrite(uint16_t pagereg, const uint8_t *data, uint8_t len,
                            void (*callback)(void *, int) = nullptr, void *user = nullptr);

    /// @brief Generic async read helper (inline for performance).
    inline void _asyncRead(uint16_t pagereg, uint8_t *dest, uint8_t len,
                           void (*callback)(void *, int) = nullptr, void *user = nullptr);

    /// @brief Blocking write helper used by begin() setup flow.
    bool _syncWrite(uint16_t pagereg, const uint8_t *data, uint8_t len);

    /// @brief Blocking read helper used by begin() setup flow.
    bool _syncRead(uint16_t pagereg, uint8_t *dest, uint8_t len);

    // ---------------------------------------------------------------------------------------
    // Static Callbacks
    // ---------------------------------------------------------------------------------------

    /// @brief PWM row transaction callback, enqueues the next pending row if any.
    /// @param user Pointer to IS31FL3733 instance.
    /// @param status Transaction status (0 = success).
    static inline void _txnCallback(void *user, int status);

    /// @brief ABM row transaction callback, enqueues the next pending mode row if any.
    /// @param user Pointer to IS31FL3733 instance.
    /// @param status Transaction status (0 = success).
    static inline void _txnModeCallback(void *user, int status);

    /// @brief GPIO IRQ callback (triggered by IRQ pin edge).
    static inline void _irqCallback();

    /// @brief ISR completion callback (processes _lastISR after F1h read).
    /// @param user Pointer to IS31FL3733 instance.
    /// @param status Transaction status (0 = success).
    static inline void _onServiceCallback(void *user, int status);

    /// @brief Command transaction callback (triggered by SERCOM ISR).
    /// @param user Pointer to IS31FL3733 instance.
    /// @param status Transaction status (0 = success).
    static void _cmdCallback(void *user, int status);

    /// @brief Open/Short detection callback (updates LED On/Off mask).
    static void _osbCallback(void *user, int status);

    /// @brief Static wrappers for ABM completion callback dispatch (PendSV-safe entry points).
    static void _abm1CallbackWrapper();
    static void _abm2CallbackWrapper();
    static void _abm3CallbackWrapper();
};

// =========================================================================================
// Inline Method Implementations
// =========================================================================================

inline void IS31FL3733::_sendRowPWM() {
    // Don't send if PWM is locked by command chain or if transaction already in-flight
    if (_pwmLocked || _pwmTxn.txPtr)
        return;

    // Dequeue next row
    int row = _pwmPendingRows.read_char();
    if (row < 0)
        return; // Nothing to send

    // Clear enqueued bit
    _pwmEnqueued &= ~(1 << row);

    // Copy row data to transmit buffer
    memcpy(_pwmTxPtr, _pwm_matrix[row], 17);

    // Point transaction at prepared PWM row buffer
    _pwmTxn.txPtr = _pwmTxPtr;

    _ensurePage(1);

    // Enqueue transaction
    _hw->enqueueWIRE(&_pwmTxn);
}

inline void IS31FL3733::_sendRowMode() {
    // Don't send if ABM is locked by command chain or if transaction already in-flight
    if (_abmLocked || _abmTxn.txPtr)
        return;

    // Dequeue next row
    int row = _abmPendingRows.read_char();
    if (row < 0)
        return; // Nothing to send

    // Clear enqueued bit
    _abmEnqueued &= ~(1 << row);

    // Copy row data to transmit buffer
    // [row][0] is the page 2 row address (0x00..0x0B)
    memcpy(_abmTxPtr, _abm_matrix[row], 17);

    // Point transaction at prepared ABM row buffer
    _abmTxn.txPtr = _abmTxPtr;

    _ensurePage(2);

    // Enqueue transaction
    _hw->enqueueWIRE(&_abmTxn);
}

inline void IS31FL3733::_asyncWrite(uint16_t pagereg, const uint8_t *data, uint8_t len,
                                    void (*callback)(void *, int), void *user) {
    uint8_t page = pagereg >> 8;

    // Ensure page if needed (pages 0-3)
    if (page < 4) {
        _ensurePage(page);
        _cmdTx[0] = pagereg & 0xFF;
    } else if (page >= IMR)
        _cmdTx[0] = page;
    else
        return;

    // Copy data to TX buffer
    memcpy(_cmdTx + 1, data, len);

    // Configure and enqueue write transaction
    _cmdTxn[2].length = 1 + len;
    _cmdCtx[2].userCallback = callback;
    _cmdCtx[2].user = user;
    _cmdCtx[2].isFinal = true; // This is the only transaction
    _cmdCtx[2].initialStatus = 0;
    _cmdTxn[2].onComplete = _cmdCallback;
    _cmdTxn[2].user = &_cmdCtx[2];
    _hw->enqueueWIRE(&_cmdTxn[2]);
}

inline void IS31FL3733::_asyncRead(uint16_t pagereg, uint8_t *dest, uint8_t len,
                                   void (*callback)(void *, int), void *user) {
    uint8_t page = pagereg >> 8;

    // Ensure page if needed
    if (page < 4) {
        _ensurePage(page);
        _cmdTx[0] = pagereg & 0xFF;
    } else if (page >= IMR)
        _cmdTx[0] = page;
    else
        return;

    // Configure write transaction (send register address) - NOT final
    _cmdTxn[2].length = 1;
    _cmdCtx[2].userCallback = nullptr;
    _cmdCtx[2].user = nullptr;
    _cmdCtx[2].isFinal = false; // Address phase doesn't trigger callback
    _cmdCtx[2].initialStatus = 0;
    _cmdTxn[2].onComplete = _cmdCallback;
    _cmdTxn[2].user = &_cmdCtx[2];

    // Configure read transaction - IS final (will invoke user callback)
    _cmdTxn[3].rxPtr = dest;
    _cmdTxn[3].length = len;
    _cmdCtx[3].userCallback = callback;
    _cmdCtx[3].user = user;
    _cmdCtx[3].isFinal = true;    // Data phase is final, triggers callback
    _cmdCtx[3].initialStatus = 0; // Can be populated by write phase if needed
    _cmdTxn[3].onComplete = _cmdCallback;
    _cmdTxn[3].user = &_cmdCtx[3];

    // Enqueue both transactions
    _hw->enqueueWIRE(&_cmdTxn[2]);
    _hw->enqueueWIRE(&_cmdTxn[3]);
}

// ---------------------------------------------------------------------------------------
// Inline Callbacks
// ---------------------------------------------------------------------------------------

inline void IS31FL3733::_txnCallback(void *user, int status) {
    IS31FL3733 *self = (IS31FL3733 *)user;

    // Clear PWM transaction in-progress flag
    self->_pwmTxn.txPtr = nullptr;

    // Transaction complete - send next PWM row if available
    self->_sendRowPWM();
}

inline void IS31FL3733::_txnModeCallback(void *user, int status) {
    IS31FL3733 *self = (IS31FL3733 *)user;

    // Clear ABM transaction in-progress flag
    self->_abmTxn.txPtr = nullptr;

    // Transaction complete - send next ABM row if available
    self->_sendRowMode();
}

inline void IS31FL3733::_irqCallback() {
    // Hardware ISR: Read F1h (ISR register) into _lastISR, with _onServiceCallback as completion
    // No page select needed for common register F1h
    if (_instance)
        _instance->_asyncRead((uint16_t)ISR << 8, &_instance->_lastISR, 1, _onServiceCallback,
                              _instance);
}

inline void IS31FL3733::_onServiceCallback(void *user, int status) {
    // Static callback after ISR read completes - dispatch to instance method
    IS31FL3733 *self = (IS31FL3733 *)user;
    // Process _lastISR value (already read by _irqCallback)
    // Lock PWM writes during fault handling
    self->_pwmLocked = true;
    self->_abmLocked = true;

    uint8_t isr = self->_lastISR;

    if (isr & 0x1C) { // ABM1 (0x04), ABM2 (0x08), ABM3 (0x10)
        // Dispatch ABM completion to registered callback(s)
        if (isr & ISR_ABM1)
            _abm1CallbackWrapper();
        if (isr & ISR_ABM2)
            _abm2CallbackWrapper();
        if (isr & ISR_ABM3)
            _abm3CallbackWrapper();
    }

    // Check for open/short faults (bits 0-1)
    if (isr & 0x3) { // OB (0x01) or SB (0x02)
        // Determine which fault register to read
        uint16_t pagereg = (isr & ISR_OB) ? LEDOPEN : LEDSHORT;
        uint8_t *dest = (isr & ISR_OB) ? self->_ledOpen : self->_ledShort;

        // Read fault register asynchronously
        self->_asyncRead(pagereg, dest, 24, _osbCallback, self);

        return;
    }

    // No recognized interrupts - just unlock PWM/ABM
    self->_resumeDataTransfers();
}

} // namespace IS31FL3733
