#pragma once
#ifndef IR_RECEIVE_HPP
#define IR_RECEIVE_HPP

#include <Arduino.h>
#include "IRremoteInt.h"
#include "CppList.hpp"
#include "MultiIRShared.h"
#include "private/IRTimer.hpp"
#include "driver/gpio.h"

#ifndef IR_INVALID_PIN
#define IR_INVALID_PIN 0xFF
#endif

/**
 * @brief Clamp a 16-bit tick count to the configured raw-buffer element size.
 */
static inline RAWBUF_DATA_TYPE clampTicksToRawbuf(uint16_t ticks) {
    if (sizeof(RAWBUF_DATA_TYPE) == 1 && ticks > UINT8_MAX) {
        return static_cast<RAWBUF_DATA_TYPE>(UINT8_MAX);
    }

    return static_cast<RAWBUF_DATA_TYPE>(ticks);
}

// ============================================================================
// IRrecv construction and configuration
// ============================================================================

IRrecv::IRrecv() {
    decodedIRData.rawDataPtr = &irparams;

    irparams.IRReceivePin     = IR_INVALID_PIN;
    irparams.StateForISR      = IR_REC_STATE_IDLE;
    irparams.TickCounterForISR = 0;
    irparams.OverflowFlag     = false;
    irparams.rawlen           = 0;

    isActive = false;
}

IRrecv::IRrecv(uint8_t receivePin)
    : IRrecv() {
    setReceivePin(receivePin);
}

IRrecv::IRrecv(uint8_t receivePin, uint8_t feedbackLedPin)
    : IRrecv() {
    setReceivePin(receivePin);
    setLEDFeedback(feedbackLedPin, false);
}

void IRrecv::setReceivePin(uint8_t receivePin) {
    if (receivePin == IR_INVALID_PIN) {
        return;
    }

    irparams.IRReceivePin = receivePin;
    pinMode(receivePin, INPUT_PULLUP);
}

void IRrecv::begin(uint8_t receivePin,
                   bool enableLedFeedback,
                   uint8_t feedbackLedPin) {
    setReceivePin(receivePin);
    setLEDFeedback(feedbackLedPin, enableLedFeedback);

    noInterrupts();
    const bool registered = IrReceiverRegistry().Add(this);
    interrupts();

    if (registered) {
        enableIRIn();
    }
}

void IRrecv::start() {
    enableIRIn();
}

void IRrecv::start(uint32_t microsecondsToAddToGapCounter) {
    enableIRIn();

    const uint32_t additionalTicks =
        microsecondsToAddToGapCounter / MICROS_PER_TICK;

    noInterrupts();

    const uint32_t updatedCounter =
        static_cast<uint32_t>(irparams.TickCounterForISR) + additionalTicks;

    irparams.TickCounterForISR =
        updatedCounter > UINT16_MAX
            ? UINT16_MAX
            : static_cast<uint16_t>(updatedCounter);

    interrupts();
}

void IRrecv::stop() {
    disableIRIn();
}

void IRrecv::end() {
    disableIRIn();

    noInterrupts();
    IrReceiverRegistry().Remove(this);
    interrupts();
}

// ============================================================================
// Receiver and shared-timer lifecycle
// ============================================================================

void IRrecv::enableIRIn() {
    if (irparams.IRReceivePin == IR_INVALID_PIN || isActive) {
        return;
    }

    noInterrupts();

    if (gActiveReceivers == 0) {
        if (!timerConfigForReceive()) {
            interrupts();
            return;
        }

        TIMER_ENABLE_RECEIVE_INTR;
    }

    irparams.StateForISR       = IR_REC_STATE_IDLE;
    irparams.TickCounterForISR = 0;
    irparams.OverflowFlag      = false;
    irparams.rawlen            = 0;

    isActive = true;
    ++gActiveReceivers;

    interrupts();

    pinMode(irparams.IRReceivePin, INPUT_PULLUP);
}

void IRrecv::disableIRIn() {
    noInterrupts();

    if (isActive) {
        isActive = false;

        if (gActiveReceivers > 0) {
            --gActiveReceivers;
        }

        if (gActiveReceivers == 0) {
            TIMER_DISABLE_RECEIVE_INTR;
        }
    }

    interrupts();
}

bool IRrecv::isIdle() {
    return irparams.StateForISR == IR_REC_STATE_IDLE;
}

bool IRrecv::available() {
    return irparams.StateForISR == IR_REC_STATE_STOP;
}

void IRrecv::resume() {
    noInterrupts();

    if (irparams.StateForISR == IR_REC_STATE_STOP) {
        irparams.rawlen       = 0;
        irparams.OverflowFlag = false;
        irparams.StateForISR  = IR_REC_STATE_IDLE;
    }

    interrupts();
}

// ============================================================================
// Decode initialization and protocol dispatch
// ============================================================================

void IRrecv::initDecodedIRData() {
    if (irparams.OverflowFlag) {
        irparams.OverflowFlag = false;
        irparams.rawlen       = 0;
        decodedIRData.flags   = IRDATA_FLAGS_WAS_OVERFLOW;
    } else {
        decodedIRData.flags = IRDATA_FLAGS_EMPTY;

        lastDecodedProtocol = decodedIRData.protocol;
        lastDecodedCommand  = decodedIRData.command;
        lastDecodedAddress  = decodedIRData.address;
    }

    decodedIRData.protocol       = UNKNOWN;
    decodedIRData.command        = 0;
    decodedIRData.address        = 0;
    decodedIRData.decodedRawData = 0;
    decodedIRData.numberOfBits   = 0;
}

IRData *IRrecv::read() {
    if (!available()) {
        return nullptr;
    }

    return decode() ? &decodedIRData : nullptr;
}

bool IRrecv::decode() {
    if (!available()) {
        return false;
    }

    initDecodedIRData();

    if (decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
        decodedIRData.protocol = UNKNOWN;
        return true;
    }

#if defined(DECODE_NEC)
    if (decodeNEC()) return true;
#endif

#if defined(DECODE_PANASONIC) || defined(DECODE_KASEIKYO)
    if (decodeKaseikyo()) return true;
#endif

#if defined(DECODE_DENON)
    if (decodeDenon()) return true;
#endif

#if defined(DECODE_SONY)
    if (decodeSony()) return true;
#endif

#if defined(DECODE_RC5)
    if (decodeRC5()) return true;
#endif

#if defined(DECODE_RC6)
    if (decodeRC6()) return true;
#endif

#if defined(DECODE_LG)
    if (decodeLG()) return true;
#endif

#if defined(DECODE_JVC)
    if (decodeJVC()) return true;
#endif

#if defined(DECODE_SAMSUNG)
    if (decodeSamsung()) return true;
#endif

#if defined(DECODE_WHYNTER)
    if (decodeWhynter()) return true;
#endif

#if defined(DECODE_LEGO_PF)
    if (decodeLegoPowerFunctions()) return true;
#endif

#if defined(DECODE_BOSEWAVE)
    if (decodeBoseWave()) return true;
#endif

#if defined(DECODE_MAGIQUEST)
    if (decodeMagiQuest()) return true;
#endif

#if defined(DECODE_DISTANCE)
    if (decodeDistance()) return true;
#endif

#if defined(DECODE_HASH)
    if (decodeHash()) return true;
#endif

    return false;
}

// ============================================================================
// Generic pulse-width and pulse-distance decoders
// ============================================================================

bool IRrecv::decodePulseWidthData(uint8_t numberOfBits,
                                  uint8_t startOffset,
                                  uint16_t oneMarkMicros,
                                  uint16_t zeroMarkMicros,
                                  uint16_t bitSpaceMicros,
                                  bool msbFirst) {
    if (numberOfBits == 0 ||
        numberOfBits > 32 ||
        startOffset >= irparams.rawlen) {
        return false;
    }

    RAWBUF_DATA_TYPE *raw = &irparams.rawbuf[startOffset];
    RAWBUF_DATA_TYPE *end = &irparams.rawbuf[irparams.rawlen];

    uint32_t decodedData = 0;

    for (uint8_t bitIndex = 0; bitIndex < numberOfBits; ++bitIndex) {
        if (raw >= end) {
            return false;
        }

        bool bitValue;

        if (matchMark(*raw, oneMarkMicros)) {
            bitValue = true;
        } else if (matchMark(*raw, zeroMarkMicros)) {
            bitValue = false;
        } else {
            return false;
        }

        ++raw;

        if (msbFirst) {
            decodedData = (decodedData << 1) | (bitValue ? 1U : 0U);
        } else if (bitValue) {
            decodedData |= (1UL << bitIndex);
        }

        const bool isLastBit = (bitIndex + 1U == numberOfBits);

        if (!isLastBit || raw < end) {
            if (raw >= end || !matchSpace(*raw, bitSpaceMicros)) {
                return false;
            }

            ++raw;
        }
    }

    decodedIRData.decodedRawData = decodedData;
    return true;
}

bool IRrecv::decodePulseDistanceData(uint8_t numberOfBits,
                                     uint8_t startOffset,
                                     uint16_t bitMarkMicros,
                                     uint16_t oneSpaceMicros,
                                     uint16_t zeroSpaceMicros,
                                     bool msbFirst) {
    if (numberOfBits == 0 ||
        numberOfBits > 32 ||
        startOffset >= irparams.rawlen) {
        return false;
    }

    RAWBUF_DATA_TYPE *raw = &irparams.rawbuf[startOffset];
    RAWBUF_DATA_TYPE *end = &irparams.rawbuf[irparams.rawlen];

    uint32_t decodedData = 0;

    for (uint8_t bitIndex = 0; bitIndex < numberOfBits; ++bitIndex) {
        if (raw >= end || !matchMark(*raw, bitMarkMicros)) {
            return false;
        }

        ++raw;

        if (raw >= end) {
            return false;
        }

        bool bitValue;

        if (matchSpace(*raw, oneSpaceMicros)) {
            bitValue = true;
        } else if (matchSpace(*raw, zeroSpaceMicros)) {
            bitValue = false;
        } else {
            return false;
        }

        ++raw;

        if (msbFirst) {
            decodedData = (decodedData << 1) | (bitValue ? 1U : 0U);
        } else if (bitValue) {
            decodedData |= (1UL << bitIndex);
        }
    }

    decodedIRData.decodedRawData = decodedData;
    return true;
}

// ============================================================================
// Biphase decoder helpers
// ============================================================================

uint8_t sBiphaseDecodeRawbuffOffset;

static uint16_t sCurrentTimingIntervals;
static uint16_t sBiphaseTimeUnit;
static uint8_t sUsedTimingIntervals;

void IRrecv::initBiphaselevel(uint8_t rawBufferOffset,
                              uint16_t biphaseTimeUnit) {
    sBiphaseDecodeRawbuffOffset = rawBufferOffset;
    sBiphaseTimeUnit            = biphaseTimeUnit;
    sUsedTimingIntervals        = 0;
}

uint8_t IRrecv::getBiphaselevel() {
    if (sBiphaseDecodeRawbuffOffset >= irparams.rawlen) {
        return SPACE;
    }

    const uint8_t level = sBiphaseDecodeRawbuffOffset & 1U;

    if (sUsedTimingIntervals == 0) {
        const uint16_t currentTicks =
            irparams.rawbuf[sBiphaseDecodeRawbuffOffset];

        const int16_t correction =
            level == MARK ? MARK_EXCESS_MICROS : -MARK_EXCESS_MICROS;

        if (matchTicks(currentTicks, sBiphaseTimeUnit + correction)) {
            sCurrentTimingIntervals = 1;
        } else if (matchTicks(
                       currentTicks,
                       (2U * sBiphaseTimeUnit) + correction)) {
            sCurrentTimingIntervals = 2;
        } else if (matchTicks(
                       currentTicks,
                       (3U * sBiphaseTimeUnit) + correction)) {
            sCurrentTimingIntervals = 3;
        } else {
            return 0xFF;
        }
    }

    ++sUsedTimingIntervals;

    if (sUsedTimingIntervals >= sCurrentTimingIntervals) {
        sUsedTimingIntervals = 0;
        ++sBiphaseDecodeRawbuffOffset;
    }

    return level;
}

// ============================================================================
// Hash decoder
// ============================================================================

#if defined(DECODE_HASH)

uint8_t IRrecv::compare(unsigned int oldValue, unsigned int newValue) {
    if (newValue * 10U < oldValue * 8U) {
        return 0;
    }

    if (oldValue * 10U < newValue * 8U) {
        return 2;
    }

    return 1;
}

bool IRrecv::decodeHash() {
    if (irparams.rawlen < 6) {
        return false;
    }

    static constexpr uint32_t FNV_PRIME = 16777619UL;
    static constexpr uint32_t FNV_BASIS = 2166136261UL;

    uint32_t hash = FNV_BASIS;

    for (uint16_t index = 1; index + 2U < irparams.rawlen; ++index) {
        hash = (hash * FNV_PRIME) ^
               compare(irparams.rawbuf[index],
                       irparams.rawbuf[index + 2U]);
    }

    decodedIRData.decodedRawData = hash;
    decodedIRData.numberOfBits   = 32;
    decodedIRData.protocol       = UNKNOWN;

    return true;
}

#endif

// ============================================================================
// Timing comparison helpers
// ============================================================================

bool matchTicks(uint16_t measuredTicks, uint16_t desiredMicros) {
    return measuredTicks >= TICKS_LOW(desiredMicros) &&
           measuredTicks <= TICKS_HIGH(desiredMicros);
}

bool MATCH(uint16_t measuredTicks, uint16_t desiredMicros) {
    return matchTicks(measuredTicks, desiredMicros);
}

bool matchMark(uint16_t measuredTicks, uint16_t desiredMicros) {
    const uint16_t compensatedMicros =
        desiredMicros + MARK_EXCESS_MICROS;

    return measuredTicks >= TICKS_LOW(compensatedMicros) &&
           measuredTicks <= TICKS_HIGH(compensatedMicros);
}

bool MATCH_MARK(uint16_t measuredTicks, uint16_t desiredMicros) {
    return matchMark(measuredTicks, desiredMicros);
}

bool matchSpace(uint16_t measuredTicks, uint16_t desiredMicros) {
    if (desiredMicros < MARK_EXCESS_MICROS) {
        return false;
    }

    const uint16_t compensatedMicros =
        desiredMicros - MARK_EXCESS_MICROS;

    return measuredTicks >= TICKS_LOW(compensatedMicros) &&
           measuredTicks <= TICKS_HIGH(compensatedMicros);
}

bool MATCH_SPACE(uint16_t measuredTicks, uint16_t desiredMicros) {
    return matchSpace(measuredTicks, desiredMicros);
}

int getMarkExcessMicros() {
    return MARK_EXCESS_MICROS;
}

int getMICROS_PER_TICK() {
    return MICROS_PER_TICK;
}

// ============================================================================
// Receiver state machine used by the timer ISR
// ============================================================================

static inline void discardFrame(irparams_struct &params) {
    params.OverflowFlag      = false;
    params.rawlen            = 0;
    params.TickCounterForISR = 0;
    params.StateForISR       = IR_REC_STATE_IDLE;
}

static inline void processReceiver(irparams_struct &params,
                                   uint8_t inputLevel) {
    if (params.TickCounterForISR < UINT16_MAX) {
        ++params.TickCounterForISR;
    }

    switch (params.StateForISR) {
        case IR_REC_STATE_IDLE:
            if (inputLevel == INPUT_MARK) {
                if (params.TickCounterForISR > RECORD_GAP_TICKS) {
                    params.OverflowFlag = false;
                    params.rawbuf[0] =
                        clampTicksToRawbuf(params.TickCounterForISR);
                    params.rawlen      = 1;
                    params.StateForISR = IR_REC_STATE_MARK;
                }

                params.TickCounterForISR = 0;
            }
            break;

        case IR_REC_STATE_MARK:
            if (inputLevel != INPUT_MARK) {
                if (params.TickCounterForISR < MIN_SIGNAL_TICKS ||
                    params.rawlen >= RAW_BUFFER_LENGTH) {
                    discardFrame(params);
                    break;
                }

                params.rawbuf[params.rawlen++] =
                    clampTicksToRawbuf(params.TickCounterForISR);

                params.TickCounterForISR = 0;
                params.StateForISR       = IR_REC_STATE_SPACE;
            }
            break;

        case IR_REC_STATE_SPACE:
            if (inputLevel == INPUT_MARK) {
                if (params.TickCounterForISR < MIN_SIGNAL_TICKS ||
                    params.rawlen >= RAW_BUFFER_LENGTH) {
                    discardFrame(params);
                    break;
                }

                params.rawbuf[params.rawlen++] =
                    clampTicksToRawbuf(params.TickCounterForISR);

                params.TickCounterForISR = 0;
                params.StateForISR       = IR_REC_STATE_MARK;
            } else if (params.TickCounterForISR > RECORD_GAP_TICKS) {
                params.StateForISR = IR_REC_STATE_STOP;
            }
            break;

        case IR_REC_STATE_STOP:
            if (inputLevel == INPUT_MARK) {
                params.TickCounterForISR = 0;
            }
            break;

        default:
            discardFrame(params);
            break;
    }
}

// ============================================================================
// Shared ESP32 timer interrupt handler
// ============================================================================

ARDUINO_ISR_ATTR void IRTimerInterruptHandler() {
    for (uint8_t index = 0;
         index < IrReceiverRegistry().Capacity();
         ++index) {
        IRrecv *receiver = IrReceiverRegistry().GetItem(index);

        if (receiver == nullptr ||
            !receiver->isActive ||
            receiver->irparams.IRReceivePin == IR_INVALID_PIN) {
            continue;
        }

        const uint8_t inputLevel = static_cast<uint8_t>(
            gpio_get_level(
                static_cast<gpio_num_t>(receiver->irparams.IRReceivePin)));

        processReceiver(receiver->irparams, inputLevel);
    }
}

#endif // IR_RECEIVE_HPP
