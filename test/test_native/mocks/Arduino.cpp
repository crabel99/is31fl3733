#include "Arduino.h"

int g_pinModeCalls = 0;
int g_digitalWriteCalls = 0;
int g_attachInterruptCalls = 0;
int g_detachInterruptCalls = 0;
int g_delayCalls = 0;
unsigned long g_mockMillis = 0;
uint8_t g_lastPinModePin = 0;
uint8_t g_lastPinModeMode = 0;
uint8_t g_lastDigitalWritePin = 0;
uint8_t g_lastDigitalWriteValue = 0;
int g_lastAttachInterruptNum = -1;
int g_lastAttachInterruptMode = -1;
int g_lastDetachInterruptNum = -1;
