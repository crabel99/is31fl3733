#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef INPUT
#define INPUT 0x0
#endif

#ifndef OUTPUT
#define OUTPUT 0x1
#endif

#ifndef LOW
#define LOW 0x0
#endif

#ifndef HIGH
#define HIGH 0x1
#endif

#ifndef FALLING
#define FALLING 0x2
#endif

extern int g_pinModeCalls;
extern int g_digitalWriteCalls;
extern int g_attachInterruptCalls;
extern int g_detachInterruptCalls;
extern int g_delayCalls;
extern unsigned long g_mockMillis;
extern uint8_t g_lastPinModePin;
extern uint8_t g_lastPinModeMode;
extern uint8_t g_lastDigitalWritePin;
extern uint8_t g_lastDigitalWriteValue;
extern int g_lastAttachInterruptNum;
extern int g_lastAttachInterruptMode;
extern int g_lastDetachInterruptNum;

inline void resetArduinoMockState() {
    g_pinModeCalls = 0;
    g_digitalWriteCalls = 0;
    g_attachInterruptCalls = 0;
    g_detachInterruptCalls = 0;
    g_delayCalls = 0;
    g_mockMillis = 0;
    g_lastPinModePin = 0;
    g_lastPinModeMode = 0;
    g_lastDigitalWritePin = 0;
    g_lastDigitalWriteValue = 0;
    g_lastAttachInterruptNum = -1;
    g_lastAttachInterruptMode = -1;
    g_lastDetachInterruptNum = -1;
}

inline void pinMode(uint8_t pin, uint8_t mode) {
    g_pinModeCalls++;
    g_lastPinModePin = pin;
    g_lastPinModeMode = mode;
}
inline void digitalWrite(uint8_t pin, uint8_t value) {
    g_digitalWriteCalls++;
    g_lastDigitalWritePin = pin;
    g_lastDigitalWriteValue = value;
}
inline int digitalPinToInterrupt(uint8_t pin) {
    return static_cast<int>(pin);
}
inline void attachInterrupt(int interruptNum, void (*)(), int mode) {
    g_attachInterruptCalls++;
    g_lastAttachInterruptNum = interruptNum;
    g_lastAttachInterruptMode = mode;
}
inline void detachInterrupt(int interruptNum) {
    g_detachInterruptCalls++;
    g_lastDetachInterruptNum = interruptNum;
}
inline void delay(unsigned long) {
    g_delayCalls++;
}

inline unsigned long millis() {
    return g_mockMillis++;
}
