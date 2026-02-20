<!-- markdownlint-disable MD036 -->
# IS31FL3733 Async DMA I2C Architecture

## Overview

This document describes an elegant, fully asynchronous DMA-driven I2C communication architecture for the IS31FL3733 LED matrix driver. The design eliminates blocking I2C operations by leveraging SERCOM (hardware I2C controller) callbacks, double-buffered row updates, and a lock-free ring buffer for pending transactions.

ABM support is intentionally abandoned in this architecture.

## Problem Statement

Traditional I2C LED driver libraries block the main program during register writes, especially when updating multiple rows or the entire 12×16 matrix. This architecture solves that by:

- Moving all I2C traffic to a background async engine
- Using DMA for efficient data transfers
- Maintaining lock-free data structures to avoid CPU overhead
- Supporting real-time color updates without blocking

## Architecture Components

### Class Structure

```cpp
class IS31FL3733 {
public:
    // Public API
    bool begin(TwoWire &wire, uint8_t addr = 0x50, ColorOrder order = GRB, uint8_t sdbPin = 0xFF, uint8_t irqPin = 0xFF);
    void DeviceOn();
    void DeviceOff();
    void SetPixelPWM(uint8_t row, uint8_t col, uint8_t value = 0);
    void SetPixelColor(uint8_t row, uint8_t col, uint32_t rgb = 0);
    void SetRowPWM(uint8_t row, uint8_t value = 0);
    void SetRowColor(uint8_t row, uint32_t rgb = 0);
    void Fill(uint32_t rgb = 0);

private:
    // Private state
    SERCOM *_hw;                  // SERCOM hardware instance
    SercomTxn _txn;               // Single in-flight transaction descriptor
    uint8_t _txPtr[17];           // Transaction buffer [reg_addr + 16 PWM bytes]
    uint8_t _pwm_matrix[12][17];  // [0]=row addr, [1..16]=PWM for CS1..CS16
    RingBufferN<12> _rowRingBuffer;
    Enqueued _enqueued;           // Bitfield tracking queued rows
    ColorOrder _colorOrder;       // GRB, RGB, etc.
    uint8_t _sdbPin;              // Shutdown pin (0xFF if unused)
    uint8_t _irqPin;              // Interrupt pin (0xFF if unused)
    bool _pwmLocked;              // True while non-PWM chain is running (blocks PWM sends)

    // Fault detection cache (LED Open/Short registers)
    uint8_t _ledOpen[24];
    uint8_t _ledShort[24];
    uint8_t _ledOn[24];

    // Command transaction chain (non-PWM operations: page select, read, write)
    SercomTxn _cmdTxn[4];        // [0]=unlock, [1]=page, [2]=read/write, [3]=read/write
    uint8_t _crwlTx[2] = {0xFE, 0xC5};
    uint8_t _pgSelTx[2];
    uint8_t _cmdTx[25];          // TX buffer (max: 1 addr + 24 data for LED On/Off)
    uint8_t _cmdRx[24];          // RX buffer (max: 24 bytes for LED Open/Short)

    // Private methods
    void _sendRow();              // Dequeue and transmit next row
    void _selectPage(uint8_t page);  // Unlock + select page (enqueues _cmdTxn[0..1])
    void _unlockPwm();            // Restore Page 1, clear _pwmLocked, kick _sendRow()
    void _onService();            // GPIO ISR handler -> determines service path (OB/SB/ABM)
    static void _txnCallback(void *user, int status);  // PWM row callback
    static void _irqCallback();        // GPIO ISR -> calls _onService()
    static void _cmdCallback(void *user, int status);  // Command chain callback
};
```

### 1. PWM Matrix: 12×17 Row Buffer

```cpp
uint8_t _pwm_matrix[12][17]  // [0]=row addr, [1..16]=PWM for CS1..CS16
```

- **Private member** holding all LED PWM state
- Byte 0 is the Page 1 row address (0x00, 0x10, ..., 0xB0)
- Bytes 1–16 are PWM intensity (0–255) for one row
- The same row buffer doubles as the transaction payload source

### 2. SERCOM Transaction Descriptor

```cpp
// From SERCOM_txn.h
struct SercomTxn {
    uint16_t config;         // Common + protocol-specific flags/fields
    uint16_t address;        // I2C addr, SPI CS/baud, UART RTS/CTS
    size_t length;
    const uint8_t *txPtr;
    uint8_t *rxPtr;
    void (*onComplete)(void *user, int status);
    void *user;
};

SercomTxn _txn;       // Private: single in-flight transaction
uint8_t _txPtr[17];   // Private: transaction buffer
```

- **Private members**
- Each transaction is **17 bytes**: Page 1 row address + 16 PWM values
- `_txn.txPtr` points to `_txPtr` when a transaction is active, `nullptr` when idle

### 3. Row Ring Buffer

```cpp
RingBufferN<12> _rowRingBuffer;
```

- **Private member**
- Holds row numbers (0–11) that need updating
- Main program enqueues row number when a row is modified
- Background callback dequeues and transmits
- Repeated writes to the same row update `_pwm_matrix` but do not add duplicate ring entries

### 4. Enqueued Bitfield

```cpp
struct Enqueued {
    uint16_t data;  // Bitfield: bit N = row N is currently queued
    inline uint8_t get(uint8_t n) { return (data >> n) & 1; }
    inline void set(uint8_t n) { data |= (1 << n); }
    inline void clear(uint8_t n) { data &= ~(1 << n); }
};

Enqueued _enqueued;  // Private member
```

- **Private member**
- Set when a row is enqueued
- Cleared when a row is dequeued for transmission
- If a row is modified while in-flight, it is re-enqueued via this bitfield

### 5. Async/Sync I/O Helpers

Unified infrastructure for register access with configurable callback behavior:

```cpp
// Inline helpers (defined in .cpp for performance)
inline void _asyncWrite(uint16_t pagereg, const uint8_t *data, uint8_t len,
                       void (*callback)(void *, int) = nullptr, void *user = nullptr);

inline void _asyncRead(uint16_t pagereg, uint8_t *dest, uint8_t len,
                      void (*callback)(void *, int) = nullptr, void *user = nullptr);
```

**Design Features**:

- **Page encoding**: High byte = page (0-3) or common register (0xFE, 0xF0, 0xF1)
- **Auto page-select**: Pages 0-3 trigger `_selectPage()`, common registers skip it
- **Callback support**: Optional callback for async completion notification
- **Reuses command chain**: Always uses `_cmdTxn[2]` for write, `_cmdTxn[3]` for read
- **Inline for performance**: Minimal overhead in ISR context

**Usage Patterns**:

```cpp
// Async with callback (ISR-safe)
_asyncRead((uint16_t)ISR << 8, &_lastISR, 1, isrCallback, this);

// Async fire-and-forget
_asyncWrite(LEDONOFF, _ledOn, 24, nullptr, nullptr);

// Synchronous (blocking)
auto syncCallback = [](void *user, int status) {
    volatile bool *flag = (volatile bool *)user;
    *flag = true;
};
volatile bool syncComplete = false;
_asyncWrite(CR, &cr_value, 1, syncCallback, (void *)&syncComplete);
while (!syncComplete);
```

### 6. Initialization {#arch-initialization}

```cpp
bool IS31FL3733Driver::begin(TwoWire &wire, uint8_t addr, ColorOrder order, uint8_t sdbPin, uint8_t irqPin)
{
    _hw = wire.getSercom();    // Get SERCOM hardware instance
    if (_hw == nullptr)
        return false;

    _colorOrder = order;
    _sdbPin = sdbPin;
    _irqPin = irqPin;
    _rowRingBuffer.flush();    // Ensure ring buffer is empty

    // Initialize _pwm_matrix[row][0] with Page 1 row addresses
    for (uint8_t row = 0; row < 12; row++) 
        _pwm_matrix[row][0] = row * 0x10;  // 0x00, 0x10, ..., 0xB0

    // Configure transaction descriptors
    
    // CRWL Txn (unlock)
    _cmdTxn[0].config = I2C_CFG_STOP;
    _cmdTxn[0].address = addr;
    _cmdTxn[0].length = 2;
    _cmdTxn[0].txPtr = _crwlTx;
    _cmdTxn[0].user = this;

    // Page Selection Txn
    _cmdTxn[1].config = I2C_CFG_STOP;
    _cmdTxn[1].address = addr;
    _cmdTxn[1].length = 2;
    _cmdTxn[1].txPtr = _pgSelTx;
    _cmdTxn[1].user = this;

    // Command Write Txn
    _cmdTxn[2].config = I2C_CFG_STOP;
    _cmdTxn[2].address = addr;
    _cmdTxn[2].txPtr = _cmdTx;
    _cmdTxn[2].user = this;

    // Command Read Txn
    _cmdTxn[3].config = I2C_CFG_READ | I2C_CFG_STOP;
    _cmdTxn[3].address = addr;
    _cmdTxn[3].txPtr = _cmdRx;
    _cmdTxn[3].user = this;

    // PWM Txn
    _txn.config = I2C_CFG_STOP;
    _txn.address = addr;
    _txn.length = 17;
    _txn.txPtr = nullptr;      // Idle state
    _txn.onComplete = _txnCallback;
    _txn.user = this;          // Pass instance pointer to callback

    // Optional SDB pin control (active high)
    if (_sdbPin != 0xFF) {
        pinMode(_sdbPin, OUTPUT);
        digitalWrite(_sdbPin, HIGH);
    }

    // DeviceOn/DeviceOff toggle the SDB pin when provided; no default pin is assumed.

    // Optional IRQ pin + open/short detection
    if (_irqPin != 0xFF) {
        pinMode(_irqPin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(_irqPin), _irqCallback, FALLING);
        // Enable OSD (open/short detection) and unmask interrupts
        // Page 3: CR |= CR_OSD
        // Page 3: IMR |= IMR_IO | IMR_IS
    }

    // Synchronous device setup (blocks briefly)
    // Use blocking I2C writes for initial configuration (CR, IMR, GCC, LED On/Off)
    // This uses simplified sync helpers that wrap the async infrastructure:
    
    // Sync helpers are lambdas in begin() that use _asyncWrite/_asyncRead:
    auto syncCallback = [](void *user, int status) {
        volatile bool *flag = (volatile bool *)user;
        *flag = true;
    };
    
    auto syncWrite = [this, &syncComplete, &syncCallback](uint16_t pagereg, 
                                                           const uint8_t *data, uint8_t len) {
        syncComplete = false;
        _asyncWrite(pagereg, data, len, syncCallback, (void *)&syncComplete);
        while (!syncComplete);  // Busy-wait for completion
    };
    
    auto syncRead = [this, &syncComplete, &syncCallback](uint16_t pagereg,
                                                          uint8_t *dest, uint8_t len) {
        syncComplete = false;
        _asyncRead(pagereg, dest, len, syncCallback, (void *)&syncComplete);
        while (!syncComplete);  // Busy-wait for completion
    };
    
    // Page 3: Set CR (enable chip, optionally enable OSD)
    // Page 3: Set GCC (global current control)
    // Page 3: Set IMR (unmask interrupts if _irqPin != 0xFF)
    // Page 0: Read LED Open/Short status
    // Page 0: Set LED On/Off (all ON, minus any faulty LEDs)
    
    // Set default page to Page 1 (PWM) so all normal writes run without page switching
    _selectPage(1);  // This is also synchronous during begin()
    
    // Initialize all PWM to zero asynchronously
    Fill();

    return true;
}
}

### 7. Interrupt and Fault Detection (Open/Short)

When `_irqPin` is provided, the driver enables open/short detection (OSD) and registers a GPIO ISR.
The ISR schedules a pre-staged transaction chain that reads fault status and updates cached arrays
without blocking the main application.

**Page Management:**

- Default page is set to Page 1 (PWM) in `begin()` and stays there for all normal PWM writes
- PWM row writes (17-byte transactions) run bare on Page 1 without page switching overhead

**Preemption rule:**

- `_pwmLocked` is set by the GPIO ISR (or any non-PWM operation) before scheduling alternate chains
- Normal PWM send path checks `_pwmLocked` and exits before copying into `_txPtr`
- The final callback of any non-PWM chain calls `_unlockPwm()`, which:
  1. Restores Page 1 via `_selectPage(1)`
  2. Clears `_pwmLocked`
  3. Kicks `_sendRow()` to resume PWM traffic

**Register setup in `begin()`:**

- Enable OSD in CR (Page 3, register 0x00): `CR |= CR_OSD`
- Unmask interrupts in IMR (Page 3, register 0x10): `IMR |= IMR_IO | IMR_IS`
- Initialize LED On/Off (Page 0, 0x00..0x17) to all ON
- Initialize PWM (Page 1, 0x00..0xB0 rows) to zero via `Fill()`

**Transaction chain (Open/Short fault):**

1. GPIO ISR fires → `_irqCallback()` → `_onService()`
2. `_onService()` locks PWM: `_pwmLocked = true`
3. **Read ISR (Async)**: `_asyncRead(ISR << 8, &_lastISR, 1, isrCallback, this)`
   - Reads Interrupt Status Register (0xF1, 1 byte) into `_lastISR` cache
   - ISR callback dispatches based on `_lastISR` bits when read completes
4. **Dispatch based on ISR value**:
   - **OB/SB (bits 0x03)**: Merged open/short fault handling
     - Determines fault register: `pagereg = (isr & ISR_OB) ? LEDOPEN : LEDSHORT`
     - Determines destination: `dest = (isr & ISR_OB) ? _ledOpen : _ledShort`
     - Reads fault register: `_asyncRead(pagereg, dest, 24, _osbCallback, this)`
   - **ABM1/ABM2/ABM3 (bits 0x1C)**: Auto-breath completion interrupts
     - Currently just unlocks PWM (TODO: implement ABM handling)
   - **Unknown**: Unlocks PWM
5. **On fault read completion**: `_osbCallback()`
   - Updates LED mask: `_ledOn[i] = _ledOn[i] & ~(_ledOpen[i] | _ledShort[i])`
   - Writes updated mask: `_asyncWrite(LEDONOFF, _ledOn, 24, nullptr, nullptr)`
   - Unlocks PWM: `_unlockPwm()` restores Page 1 and resumes PWM writes

**Key Design Points**:
- All I/O uses unified `_asyncWrite`/`_asyncRead` inline helpers
- ISR reading is fully asynchronous with callback-based dispatch
- OB/SB paths merged into single branch (fault type determined from ISR bit)
- `_lastISR` cache enables lightweight ISR callback (no lambdas in ISR context)
- `_osbCallback` is static method for fault completion handling

All of the above uses the dedicated `_cmdTxn[4]` chain so it does not interfere with the normal row PWM `_txn` pipeline.

**Command transaction sizing:**
- `_cmdTxn[0]`: Unlock (PSWL write: 2 bytes)
- `_cmdTxn[1]`: Page select (PSR write: 2 bytes)
- `_cmdTxn[2..3]`: Read/write ops (max 25 TX, 24 RX bytes)
```

## State Machine: Transaction Lifecycle

### Write API (Public, Called by Main Program)

```cpp
void IS31FL3733Driver::SetPixelPWM(uint8_t row, uint8_t col, uint8_t value)
{
    uint8_t idx = (row - 1) & 0b1111; // the row passed in is an integer 1..16

    _pwm_matrix[idx][col] = value;
    
    // Enqueue row if not already queued
    if (!_enqueued.get(idx)) {
        _enqueued.set(idx);
        _rowRingBuffer.store(idx);
    }
    
    // Kick sender if idle
    if (_txn.txPtr == nullptr)
        _sendRow();
}

void IS31FL3733Driver::SetPixelColor(uint8_t row, uint8_t col, uint32_t rgb)
{
    row = (row - 1) & 0b11; // the row is an integer 1..4 mapped to 0..3

    // Extract colors
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8) & 0xFF;
    uint8_t b = rgb & 0xFF;
    
    // Calculate base hardware row (each RGB pixel uses 3 hardware rows)
    // baseRow will be 0, 3, 6, or 9 for rows 0-3
    // Add 1 to convert to 1-based indexing for SetPixelPWM
    uint8_t baseRow = row * 3 + 1;
    
    // Write to PWM matrix based on color order
    switch (_colorOrder) {
        case ColorOrder::RGB:
            SetPixelPWM(baseRow + 0, col, r);     // R on first row
            SetPixelPWM(baseRow + 1, col, g);     // G on second row
            SetPixelPWM(baseRow + 2, col, b);     // B on third row
            break;
        case ColorOrder::GRB:
            SetPixelPWM(baseRow + 0, col, g);     // G on first row
            SetPixelPWM(baseRow + 1, col, r);     // R on second row
            SetPixelPWM(baseRow + 2, col, b);     // B on third row
            break;
        case ColorOrder::RBG:
            SetPixelPWM(baseRow + 0, col, r);
            SetPixelPWM(baseRow + 1, col, b);
            SetPixelPWM(baseRow + 2, col, g);
            break;
        case ColorOrder::BRG:
            SetPixelPWM(baseRow + 0, col, b);
            SetPixelPWM(baseRow + 1, col, r);
            SetPixelPWM(baseRow + 2, col, g);
            break;
        case ColorOrder::GBR:
            SetPixelPWM(baseRow + 0, col, g);
            SetPixelPWM(baseRow + 1, col, b);
            SetPixelPWM(baseRow + 2, col, r);
            break;
        case ColorOrder::BGR:
            SetPixelPWM(baseRow + 0, col, b);
            SetPixelPWM(baseRow + 1, col, g);
            SetPixelPWM(baseRow + 2, col, r);
            break;
    }
}
```

### Send API (Private, Called from Write API or Callback)

```cpp
void IS31FL3733Driver::_sendRow()
{
    if (_pwmLocked)
        return;

    // Check if ring buffer has pending rows
    if (_rowRingBuffer.available() == 0)
        return;
    
    // Read and remove next row index
    uint8_t row = _rowRingBuffer.read_char();
    
    // Clear enqueued bit (allows row to be re-queued during transmission)
    _enqueued.clear(row);
    
    // Copy row data to transaction buffer
    memcpy(_txPtr, _pwm_matrix[row], 17);
    
    // Enqueue transaction to SERCOM
    _txn.txPtr = _txPtr;
    _hw->enqueueWIRE(_txn);
}

void IS31FL3733Driver::_selectPage(uint8_t page)
{
    page = page & 0b11;
    // Enqueue unlock transaction
    _hw->enqueueWIRE(_cmdTxn[0]);
    
    // Enqueue page select transaction (PSR = page)
    uint8_t pageSel[2] = {0xFD, page};
    memcpy(_pgSelTx, _pageSel, 2);
    _hw->enqueueWIRE(_cmdTxn[1]);
}

void IS31FL3733Driver::_unlockPwm()
{
    _selectPage(1);        // Restore PWM page (unlock + select)
    _pwmLocked = false;    // Unlock PWM chain
    _sendRow();            // Resume pending PWM writes
}
```

### SERCOM Callback (Private, Called from ISR on Transaction Complete)

```cpp
void IS31FL3733Driver::_txnCallback(void *user, int status)
{
    IS31FL3733 *self = (IS31FL3733 *)user;
    
    // Mark transaction complete (idle state)
    self->_txn.txPtr = nullptr;
    
    // If ring buffer has more pending rows, send next
    if (self->_rowRingBuffer.available() > 0)
        self->_sendRow();
}
```

### Flow Summary

1. **Main program writes** → Updates `_pwm_matrix[row][col]` → Sets `_enqueued` bit → Enqueues row → Kicks `_sendRow()` if idle
2. **`_sendRow()`** → Dequeues row → Clears `_enqueued` bit → Copies to `_txPtr` → Enqueues SERCOM transaction
3. **Callback fires** → Sets `_txn.txPtr = nullptr` → Calls `_sendRow()` if more rows pending
4. **While in-flight**: If row is updated again, `_enqueued` bit is re-set and row is re-queued

## Initialization

See [Architecture Components - Initialization](#arch-initialization) above for full `begin()` implementation.

## Public API (Non-blocking)

```cpp
// Set a single pixel (enqueues row if dirty)
driver.SetPixelColor(row, col, uint32_t rgb);

// Set entire row (enqueues row)
driver.SetRowRgb(row, r_pwm, g_pwm, b_pwm);

// Fill entire matrix (enqueues all dirty rows)
driver.Fill(uint32_t rgb);

// Arbitrary PWM array update (enqueues modified rows)
driver.UpdatePwm(const uint8_t pwm[12][17]);
```

All return immediately. No blocking I2C.

## Benefits

| Aspect | vs. Blocking I2C | This Architecture |
| ------ | ---------------- | ----------------- |
| Main program blocked? | Yes, during writes | No, fully async |
| CPU overhead | Proportional to data size | Minimal (memcpy + pointer swaps in callback) |
| Throughput | Sequential | DMA handles transmission in background |
| Latency | Unpredictable (I2C waits) | Predictable (buffered) |
| Real-time capable? | No | Yes (animations, PWM fading) |

## Thread Safety

- **No locks needed**: Ring buffer + `_enqueued` bitfield prevents duplicate queueing
- Main program reads/writes `_pwm_matrix` freely
- Single in-flight transaction uses `_txPtr` as the stable DMA source
- Clearing `_enqueued` bit before transmission allows row to be re-queued while in-flight
- All data structures are atomic (single bytes/pointers on ARM)
- `_txnCallback` runs in ISR context; `_sendRow()` is ISR-safe (no re-entrancy issues)

## Example: Animating a Bar Graph

```cpp
// Main program (non-blocking)
for (int step = 0; step < 256; step++) {
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 16; col++) {
            uint8_t brightness = (step + row * 64) % 256;
            driver.SetPixelColor(row, col, brightness);  // Enqueues row
        }
    }
    // All rows are now queued; background engine transmits asynchronously
    delay(10);  // Can do other work here
}
```

The I2C engine processes pending rows in the background while the main program continues.

## Future Enhancements

1. **Gamma correction during send**: Apply gamma curve while populating `txPtr`
2. **HSV support**: Convert HSV → RGB in the async path
3. **Interrupt statistics**: Track transaction completion rate, queue depth for performance monitoring

## References

- IS31FL3733B datasheet (see `docs/IS31FL3733B_DS.pdf`)
- SERCOM I2C DMA reference (SAMD21 or target MCU docs)
- ColorUtils: Gamma correction and HSV→RGB (see `src/is31fl3733_color_utils.hpp`)
