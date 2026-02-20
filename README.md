<!-- markdownlint-disable MD036 -->
# IS31FL3733 C++ Async Driver Library

This asynchronous DMA-driven IS31FL3733 LED driver for Arduino SAMD platforms features:

- **Non-blocking I2C**: DMA-driven transaction queue with automatic sequencing
- **RGB matrix support**: High-level color APIs with HSV and 32-bit RGB
- **Software gamma correction**: Perceptually linear brightness
- **Auto Breath Mode (ABM)**: Hardware-accelerated breathing effects with interrupt callbacks
- **Fault detection**: Runtime open/short detection with auto-disable
- **Zero heap allocation**: All buffers pre-allocated
- **88% logic coverage**: 53 passing unit tests

**Lineage:** Based on [Neil Enns' C++ library](https://github.com/neilenns/is31fl3733) (originally forked from [kkostyan's C library](https://github.com/kkostyan/is31fl3733)). This is a nearly complete rewrite for asynchronous DMA-driven operation.

## ⚠️ EXPERIMENTAL - Platform Requirements

**This is an EXPERIMENTAL asynchronous driver with specific platform requirements:**

### Required Hardware

- **SAMD microcontrollers ONLY** (SAMD21, SAMD51, etc.)
- Supported by [Adafruit ArduinoCore-samd](https://github.com/adafruit/ArduinoCore-samd)

### Required Framework

This driver requires the **async DMA branch** of SimIOFramework:

```text
https://github.com/crabel99/SimIOFramework/tree/sercom-async-dma
```

**Important:** The standard Arduino `Wire` library does NOT support the async/DMA features required by this driver. You must use the modified SimIOFramework branch above.

### Why Async/DMA?

Traditional blocking I2C operations freeze the CPU during LED matrix updates (~20ms per frame). This async implementation:

- Updates LEDs in the background via DMA
- Allows CPU to handle other tasks concurrently
- Provides interrupt-driven fault detection
- Queues PWM updates automatically

### Stability Warning

This is experimental software:

- API may change without notice
- Only tested on SimIO_Device_M0 hardware
- Not recommended for production use without thorough testing
- Requires understanding of async programming patterns

If you need a stable, **synchronous** driver for broader platform support, consider using the original upstream library or an earlier version of this fork.

---

## Using with PlatformIO

This library is registered with PlatformIO. Include it in your project via the PlatformIO library
manager.

**Required dependency:** Add the async DMA branch of SimIOFramework to your `platformio.ini`:

```ini
lib_deps = 
    https://github.com/crabel99/SimIOFramework.git#sercom-async-dma
```

## Using with other toolchains

Include `is31fl3733.hpp` and `is31fl3733.cpp` in your project, along with the SimIOFramework async DMA branch.

**Note:** This driver is NOT compatible with the standard Arduino `Wire` library. It requires the modified `TwoWire` implementation from SimIOFramework that supports async/DMA operations.

## Architecture

See [ASYNC_DMA_ARCHITECTURE.md](docs/ASYNC_DMA_ARCHITECTURE.md) for detailed design documentation.

**Key features:**

- Transaction queue with automatic page management
- Lock-free ring buffer for PWM updates
- Interrupt-driven fault detection
- Color order mapping for RGB matrices

## Quick Start

### Basic Driver (Low-Level PWM Control)

```C++
#include <Wire.h>  // SimIOFramework Wire, not standard Arduino Wire
#include "is31fl3733.hpp"

using namespace IS31FL3733;

const uint8_t SDB_PIN = 4;    // Shutdown pin (active high)
const uint8_t INTB_PIN = 0xFF; // No interrupt pin
const uint8_t IS31_ADDR = 0x50;

// Create driver instance (Wire from SimIOFramework)
IS31FL3733 driver(&Wire, IS31_ADDR, SDB_PIN, INTB_PIN);

void setup() {
  Wire.begin();
  
  // Initialize device
  if (!driver.begin()) {
    // Handle initialization failure
  }
  
  // Set global current control (0-255)
  driver.SetGCC(127);
  
  // Set individual LED PWM (row 1-12, col 1-16)
  driver.SetPixelPWM(1, 1, 255);  // Full brightness
  
  // Fill entire matrix
  driver.Fill(128);  // Half brightness
  
  // RGB color control (logical 4x16 RGB matrix)
  driver.SetColorOrder(ColorOrder::RGB);
  driver.SetPixelColor(1, 1, 255, 0, 0);  // Red pixel at row 1, col 1
}

void loop() {
  // Non-blocking - driver processes I2C queue automatically
}
```

### RGB Matrix Helper (High-Level Color APIs)

```C++
#include <Wire.h>  // SimIOFramework Wire, not standard Arduino Wire
#include "is31fl3733_rgb_matrix.hpp"

using namespace IS31FL3733;

// Wire from SimIOFramework with async/DMA support
IS31FL3733RgbMatrix matrix(&Wire, 0x50, 4, 0xFF, ColorOrder::RGB);

void setup() {
  Wire.begin();
  
  // Initialize with default GCC and clear to black
  if (!matrix.begin()) {
    // Handle failure
  }
  
  // 32-bit color with gamma correction
  matrix.SetPixelColor32(1, 1, 0xFF0000, true);  // Red with gamma
  matrix.FillColor32(0x00FF00, true);             // Green fill with gamma
  
  // HSV color (hue 0-65535, sat/val 0-255)
  matrix.SetPixelHSV(1, 1, 0, 255, 255, true);   // Red via HSV
  matrix.FillHSV(21845, 255, 255, true);         // Cyan fill via HSV
}

void loop() {
  static uint32_t lastUpdate = 0;
  static uint16_t hue = 0;
  
  // Non-blocking animation timing
  if (millis() - lastUpdate >= 50) {
    lastUpdate = millis();
    
    // Rainbow animation
    matrix.FillHSV(hue, 255, 200, true);
    hue += 512;
  }
}
```

### Auto Breath Mode (ABM)

```C++
#include <Wire.h>  // SimIOFramework Wire, not standard Arduino Wire
#include "is31fl3733.hpp"

using namespace IS31FL3733;

// Wire from SimIOFramework, IRQ on pin 3 for ABM completion callback
IS31FL3733 driver(&Wire, 0x50, 4, 3);

void abm_completed() {
  // Called when ABM finishes
}

void setup() {
  Wire.begin();
  
  driver.SetABMCallback(1, abm_completed);  // Register callback for ABM1
  driver.begin();
  
  driver.SetGCC(127);
  driver.Fill(128);
  
  // Configure for ABM1
  driver.SetMatrixMode(ABMMode::ABM1);
  
  ABMConfig config;
  config.T1 = T1_840MS;
  config.T2 = T2_840MS;
  config.T3 = T3_840MS;
  config.T4 = T4_840MS;
  config.Tbegin = LOOP_BEGIN_T4;
  config.Tend = LOOP_END_T3;
  config.Times = 2;  // Loop twice
  
  driver.ConfigureABM1(config);
  driver.SetIMR(IMR_IAB);  // Enable ABM interrupt
  driver.EnableABM(true);
  driver.TriggerABM();
}

void loop() {
  // Non-blocking - callback fires when ABM completes
}
```

## Examples

Two complete examples are provided in `/examples`:

- **`abm_arduino.ino`**: Auto Breath Mode with interrupt callbacks and non-blocking state machine
- **`rgb_matrix_arduino.ino`**: RGB matrix control with HSV rainbow animations, 32-bit color, and gamma correction

Both examples demonstrate non-blocking loop design compatible with the async driver architecture.

## API Reference

### Core Driver (`IS31FL3733`)

**Initialization:**

- `begin()` - Initialize device, returns true on success
- `end()` - Shutdown device and detach interrupts
- `DeviceOn()` / `DeviceOff()` - Software enable/disable

**PWM Control:**

- `SetPixelPWM(row, col, pwm)` - Set single LED (1-12 rows, 1-16 cols)
- `SetRowPWM(row, pwmValues[])` - Set entire row
- `Fill(pwm)` - Fill entire matrix

**RGB Color:**

- `SetPixelColor(row, col, r, g, b)` - Logical RGB pixel (1-4 rows, 1-16 cols)
- `SetColorOrder(ColorOrder)` - RGB/GRB/BRG/RBG/GBR/BGR mapping

**Auto Breath Mode:**

- `ConfigureABM1/2/3(ABMConfig)` - Configure ABM parameters
- `SetMatrixMode(ABMMode)` - Set PWM/ABM1/ABM2/ABM3 mode
- `EnableABM(bool)` - Enable/disable ABM
- `TriggerABM()` - Start ABM sequence
- `SetABMCallback(abmNum, callback)` - Register completion callback

**Configuration:**

- `SetGCC(value)` - Global current control (0-255)
- `SetIMR(mask)` - Interrupt mask register

### RGB Matrix Helper (`IS31FL3733RgbMatrix`)

Inherits all `IS31FL3733` methods plus:

**Initialization:**

- `begin(gcc, clear)` - Initialize with GCC value, optionally clear to black

**32-bit Color:**

- `SetPixelColor32(row, col, rgb, gamma)` - Set pixel with packed RGB
- `FillColor32(rgb, gamma)` - Fill with packed RGB

**HSV Color:**

- `SetPixelHSV(row, col, hue, sat, val, gamma)` - Set pixel with HSV
- `FillHSV(hue, sat, val, gamma)` - Fill with HSV

**Constants:**

- `kRows` - Matrix row count (4)
- `kCols` - Matrix column count (16)

### Color Utilities (`is31fl3733_color_utils.hpp`)

- `Gamma8(value)` - 8-bit gamma correction
- `Gamma32(rgb)` - 32-bit RGB gamma correction
- `ColorHSV(hue, sat, val)` - HSV to RGB conversion
- `PackRGB(r, g, b)` - Pack RGB into 32-bit value

## Testing

This project uses Unity Fixture with 53 passing native tests and 18 passing embedded tests.

**Native tests** (mocked I2C):

```bash
.venv/bin/pio test -e native -v
```

**Embedded tests** (SimIO_Device_M0 hardware):

```bash
.venv/bin/pio test -e embedded -v
```

**Coverage:** 88% logic coverage - see [LOGIC_COVERAGE_ANALYSIS.md](docs/LOGIC_COVERAGE_ANALYSIS.md)

When using Unity Fixture, PlatformIO may show `0 test cases` in summary. Treat Unity output as source of truth (e.g., `53 Tests 0 Failures 0 Ignored`).

## Memory Footprint

Per-driver instance: ~764 bytes

- PWM matrix: 204 bytes (12 rows × 17 bytes)
- LED mode matrix: 204 bytes (12 rows × 17 bytes)
- PWM transaction descriptor: ~24 bytes
- ABM transaction descriptor: ~24 bytes
- Command transaction chain: ~96 bytes (4 transactions)
- Command buffers: 49 bytes (25-byte TX + 24-byte RX)
- Ring buffers (PWM + ABM): ~32 bytes
- Fault cache: 72 bytes (open/short/on masks × 24 bytes each)
- Callback array: 12 bytes (3 ABM callbacks)
- Misc state, flags, pointers: ~47 bytes

**Zero heap allocation** - all buffers pre-allocated at construction, safe for interrupt context.

## Documentation

- [ASYNC_DMA_ARCHITECTURE.md](docs/ASYNC_DMA_ARCHITECTURE.md) - Detailed architecture and design decisions
- [IMPLEMENTATION_SUMMARY.md](docs/IMPLEMENTATION_SUMMARY.md) - Implementation details and usage patterns
- [LOGIC_COVERAGE_ANALYSIS.md](docs/LOGIC_COVERAGE_ANALYSIS.md) - Test coverage analysis (88% coverage)

## License

MIT License - See [LICENSE](LICENSE) file for details.

Color utilities derived from Adafruit NeoPixel library (LGPL-3.0).
