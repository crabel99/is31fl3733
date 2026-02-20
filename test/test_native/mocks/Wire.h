#pragma once

#include <cstddef>
#include <stdint.h>

struct SercomTxn {
    uint8_t config = 0;
    uint8_t address = 0;
    uint8_t *txPtr = nullptr;
    uint8_t *rxPtr = nullptr;
    uint8_t length = 0;
    void (*onComplete)(void *, int) = nullptr;
    void *user = nullptr;
};

enum : uint8_t {
    I2C_CFG_STOP = 0x01,
    I2C_CFG_READ = 0x02,
};

class SERCOM {
  public:
    bool autoComplete = true;
    int callbackStatus = 0;
    int failOnCallbackNumber = -1;
    int failStatus = -1;
    size_t callbackCount = 0;
    size_t enqueueCount = 0;
    SercomTxn *history[64]{};
    uint8_t txHistory[64][32]{};
    uint8_t txLenHistory[64]{};

    void resetTrace() {
        enqueueCount = 0;
        callbackCount = 0;
        for (size_t i = 0; i < 64; i++) {
            history[i] = nullptr;
            txLenHistory[i] = 0;
        }
    }

    void setAutoComplete(bool enabled) {
        autoComplete = enabled;
    }

    void setCallbackStatus(int status) {
        callbackStatus = status;
    }

    void setFailOnCallback(int callbackNumber, int status = -1) {
        failOnCallbackNumber = callbackNumber;
        failStatus = status;
    }

    void clearFailOnCallback() {
        failOnCallbackNumber = -1;
    }

    void enqueueWIRE(SercomTxn *txn) {
        if (enqueueCount < 64) {
            history[enqueueCount] = txn;
            if (txn && txn->txPtr && txn->length > 0) {
                const uint8_t copyLen = static_cast<uint8_t>(txn->length > 32 ? 32 : txn->length);
                txLenHistory[enqueueCount] = copyLen;
                for (uint8_t i = 0; i < copyLen; i++) {
                    txHistory[enqueueCount][i] = txn->txPtr[i];
                }
            }
        }
        enqueueCount++;

        if (txn && txn->onComplete) {
            if (autoComplete) {
                callbackCount++;
                int status = callbackStatus;
                if (failOnCallbackNumber > 0 &&
                    static_cast<int>(callbackCount) == failOnCallbackNumber)
                    status = failStatus;
                txn->onComplete(txn->user, status);
            }
        }
    }
};

template <size_t N> class RingBufferN {
  public:
    RingBufferN() : _head(0), _tail(0), _count(0) {}

    int read_char() {
        if (_count == 0) {
            return -1;
        }
        uint8_t value = _buffer[_tail];
        _tail = (_tail + 1) % N;
        _count--;
        return value;
    }

    bool store_char(uint8_t value) {
        if (_count == N) {
            return false;
        }
        _buffer[_head] = value;
        _head = (_head + 1) % N;
        _count++;
        return true;
    }

  private:
    uint8_t _buffer[N]{};
    size_t _head;
    size_t _tail;
    size_t _count;
};

class TwoWire {
  public:
    SERCOM *getSercom() {
        return &_sercom;
    }

  private:
    SERCOM _sercom;
};
