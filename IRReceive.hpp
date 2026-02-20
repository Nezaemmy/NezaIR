#pragma once
#ifndef IR_RECEIVE_HPP
#define IR_RECEIVE_HPP

#include <Arduino.h>
#include "IRremoteInt.h"
#include "CppList.hpp"

/** \addtogroup Receiving Receiving IR data for multiple protocols
 * @{
 */

/* -------------------------------------------------------------------------------------------------
 * MultiIR registry: function-static singleton avoids static-init-order problems
 * ------------------------------------------------------------------------------------------------- */
static inline CppList& IrParamsList() {
    static CppList list;
    return list;
}

/* -------------------------------------------------------------------------------------------------
 * Global receiver instance (kept for compatibility with standard IRremote usage)
 * ------------------------------------------------------------------------------------------------- */
IRrecv IrReceiver;

/* -------------------------------------------------------------------------------------------------
 * Shared timer enable refcount
 * The timer ISR is global on ATmega328P, so we only start/stop it when the
 * number of active receivers crosses zero.
 * ------------------------------------------------------------------------------------------------- */
static volatile uint8_t gActiveReceivers = 0;

/* -------------------------------------------------------------------------------------------------
 * Safe tick clamping for RAWBUF_DATA_TYPE
 * If RAWBUF_DATA_TYPE is uint8_t (the default), clamp tick values to 0xFF to
 * prevent silent wraparound. No-op when the type is wide enough.
 * ------------------------------------------------------------------------------------------------- */
static inline RAWBUF_DATA_TYPE clampTicksToRawbuf(uint16_t ticks) {
    if (sizeof(RAWBUF_DATA_TYPE) == 1 && ticks > 0xFF) {
        return (RAWBUF_DATA_TYPE)0xFF;
    }
    return (RAWBUF_DATA_TYPE)ticks;
}

/* -------------------------------------------------------------------------------------------------
 * IRrecv constructors
 * ------------------------------------------------------------------------------------------------- */
IRrecv::IRrecv() {
    decodedIRData.rawDataPtr = &irparams;  // bind decoded data to this instance's raw buffer
    setReceivePin(0);

#if !defined(DISABLE_LED_FEEDBACK_FOR_RECEIVE)
    setLEDFeedback(0, false);
#endif

    // Register this receiver's irparams in the global ISR list.
    // Add() is idempotent so calling this from multiple constructors is safe.
    noInterrupts();
    IrParamsList().Add(&irparams);
    interrupts();
}

IRrecv::IRrecv(uint8_t aReceivePin) : IRrecv() {
    setReceivePin(aReceivePin);
}

IRrecv::IRrecv(uint8_t aReceivePin, uint8_t aFeedbackLEDPin) : IRrecv() {
    setReceivePin(aReceivePin);
#if !defined(DISABLE_LED_FEEDBACK_FOR_RECEIVE)
    setLEDFeedback(aFeedbackLEDPin, false);
#else
    (void)aFeedbackLEDPin;
#endif
}

/* -------------------------------------------------------------------------------------------------
 * Stream-like API
 * ------------------------------------------------------------------------------------------------- */
void IRrecv::begin(uint8_t aReceivePin, bool aEnableLEDFeedback, uint8_t aFeedbackLEDPin) {
    setReceivePin(aReceivePin);
#if !defined(DISABLE_LED_FEEDBACK_FOR_RECEIVE)
    setLEDFeedback(aFeedbackLEDPin, aEnableLEDFeedback);
#else
    (void)aEnableLEDFeedback;
    (void)aFeedbackLEDPin;
#endif

    noInterrupts();
    IrParamsList().Add(&irparams);  // idempotent
    interrupts();

    enableIRIn();
}

void IRrecv::setReceivePin(uint8_t aReceivePinNumber) {
    irparams.IRReceivePin = aReceivePinNumber;
#if defined(__AVR__)
    irparams.IRReceivePinMask              = digitalPinToBitMask(aReceivePinNumber);
    irparams.IRReceivePinPortInputRegister = portInputRegister(digitalPinToPort(aReceivePinNumber));
#endif
    // INPUT_PULLUP keeps the pin HIGH (idle) when the IR receiver is
    // disconnected. Without this a floating pin oscillates randomly, causing
    // an interrupt storm that blocks the CPU (especially on ESP32).
    // Most IR receiver modules (VS1838B, TSOP4838 etc.) have open-collector
    // outputs and work correctly with a pull-up.
    pinMode(aReceivePinNumber, INPUT_PULLUP);
}

void IRrecv::start() {
    enableIRIn();
}

void IRrecv::start(uint32_t aMicrosecondsToAddToGapCounter) {
    enableIRIn();
    noInterrupts();
    irparams.TickCounterForISR += (uint16_t)(aMicrosecondsToAddToGapCounter / MICROS_PER_TICK);
    interrupts();
}

void IRrecv::stop() {
    disableIRIn();
}

void IRrecv::end() {
    stop();
}

/* -------------------------------------------------------------------------------------------------
 * enableIRIn / disableIRIn  (reference-counted timer ISR management)
 * ------------------------------------------------------------------------------------------------- */
void IRrecv::enableIRIn() {
    noInterrupts();

    // Start the shared timer ISR only when the first receiver becomes active.
    if (gActiveReceivers++ == 0) {
        timerConfigForReceive();
        TIMER_ENABLE_RECEIVE_INTR;
        TIMER_RESET_INTR_PENDING;
    }

    // Reset only this receiver's state machine; other receivers are unaffected.
    irparams.StateForISR       = IR_REC_STATE_IDLE;
    irparams.TickCounterForISR = 0;
    irparams.OverflowFlag      = false;
    irparams.rawlen            = 0;

    interrupts();

    pinMode(irparams.IRReceivePin, INPUT_PULLUP);  // Pull-up keeps pin stable when IR receiver is disconnected
}

void IRrecv::disableIRIn() {
    noInterrupts();
    // Guard against underflow if disableIRIn() is called without a matching enable.
    if (gActiveReceivers > 0 && --gActiveReceivers == 0) {
        TIMER_DISABLE_RECEIVE_INTR;
    }
    interrupts();
}

bool IRrecv::isIdle() {
    return (irparams.StateForISR == IR_REC_STATE_IDLE ||
            irparams.StateForISR == IR_REC_STATE_STOP);
}

void IRrecv::resume() {
    if (irparams.StateForISR == IR_REC_STATE_STOP) {
        irparams.StateForISR = IR_REC_STATE_IDLE;
    }
}

/* -------------------------------------------------------------------------------------------------
 * initDecodedIRData
 * ------------------------------------------------------------------------------------------------- */
void IRrecv::initDecodedIRData() {
    if (irparams.OverflowFlag) {
        irparams.OverflowFlag = false;
        irparams.rawlen       = 0;
        decodedIRData.flags   = IRDATA_FLAGS_WAS_OVERFLOW;
        DEBUG_PRINTLN(F("Overflow happened"));  // BUG FIX: was a plain string literal, not F() — wasted SRAM
    } else {
        decodedIRData.flags    = IRDATA_FLAGS_EMPTY;
        lastDecodedProtocol    = decodedIRData.protocol;
        lastDecodedCommand     = decodedIRData.command;
        lastDecodedAddress     = decodedIRData.address;
    }

    decodedIRData.protocol      = UNKNOWN;
    decodedIRData.command       = 0;
    decodedIRData.address       = 0;
    decodedIRData.decodedRawData = 0;
    decodedIRData.numberOfBits  = 0;
}

/* -------------------------------------------------------------------------------------------------
 * available / read
 * ------------------------------------------------------------------------------------------------- */
bool IRrecv::available() {
    return (irparams.StateForISR == IR_REC_STATE_STOP);
}

IRData* IRrecv::read() {
    if (irparams.StateForISR != IR_REC_STATE_STOP) return nullptr;  // prefer nullptr over NULL in C++
    if (decode()) return &decodedIRData;
    return nullptr;
}

/* -------------------------------------------------------------------------------------------------
 * decode()  —  try each enabled protocol decoder in priority order
 * ------------------------------------------------------------------------------------------------- */
bool IRrecv::decode() {
    if (irparams.StateForISR != IR_REC_STATE_STOP) return false;

    initDecodedIRData();

    if (decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
        decodedIRData.protocol = UNKNOWN;
        return true;  // overflow counts as "data available"
    }

#if defined(DECODE_NEC)
    if (decodeNEC())          return true;
#endif
#if defined(DECODE_PANASONIC) || defined(DECODE_KASEIKYO)
    if (decodeKaseikyo())     return true;
#endif
#if defined(DECODE_DENON)
    if (decodeDenon())        return true;
#endif
#if defined(DECODE_SONY)
    if (decodeSony())         return true;
#endif
#if defined(DECODE_RC5)
    if (decodeRC5())          return true;
#endif
#if defined(DECODE_RC6)
    if (decodeRC6())          return true;
#endif
#if defined(DECODE_LG)
    if (decodeLG())           return true;
#endif
#if defined(DECODE_JVC)
    if (decodeJVC())          return true;
#endif
#if defined(DECODE_SAMSUNG)
    if (decodeSamsung())      return true;
#endif
#if defined(DECODE_WHYNTER)
    if (decodeWhynter())      return true;
#endif
#if defined(DECODE_LEGO_PF)
    if (decodeLegoPowerFunctions()) return true;
#endif
#if defined(DECODE_BOSEWAVE)
    if (decodeBoseWave())     return true;
#endif
#if defined(DECODE_MAGIQUEST)
    if (decodeMagiQuest())    return true;
#endif
#if defined(DECODE_DISTANCE)
    if (decodeDistance())     return true;
#endif
#if defined(DECODE_HASH)
    if (decodeHash())         return true;
#endif

    // BUG FIX: original returned true here unconditionally even when no protocol
    // matched and DECODE_HASH was disabled — callers had no way to distinguish
    // "decoded OK" from "unknown protocol with no hash". Return false so callers
    // can react correctly (e.g. skip, log, or fall through to a raw handler).
    return false;
}

/* -------------------------------------------------------------------------------------------------
 * Pulse-width decoder
 * Bit value is encoded in the mark duration (one mark = long, zero mark = short).
 * ------------------------------------------------------------------------------------------------- */
bool IRrecv::decodePulseWidthData(uint8_t aNumberOfBits, uint8_t aStartOffset,
                                   uint16_t aOneMarkMicros, uint16_t aZeroMarkMicros,
                                   uint16_t aBitSpaceMicros, bool aMSBfirst) {

    RAWBUF_DATA_TYPE *tRawBufPointer = &decodedIRData.rawDataPtr->rawbuf[aStartOffset];
    RAWBUF_DATA_TYPE *tRawBufEnd     = &decodedIRData.rawDataPtr->rawbuf[decodedIRData.rawDataPtr->rawlen];
    uint32_t tDecodedData = 0;

    if (aMSBfirst) {
        for (uint_fast8_t i = 0; i < aNumberOfBits; i++) {
            // Bounds check before dereferencing
            if (tRawBufPointer >= tRawBufEnd) return false;
            if      (matchMark(*tRawBufPointer, aOneMarkMicros))  tDecodedData = (tDecodedData << 1) | 1;
            else if (matchMark(*tRawBufPointer, aZeroMarkMicros)) tDecodedData = (tDecodedData << 1);
            else return false;
            tRawBufPointer++;

            // Space check (optional on the very last bit)
            if (tRawBufPointer < tRawBufEnd) {
                if (!matchSpace(*tRawBufPointer, aBitSpaceMicros)) return false;
                tRawBufPointer++;
            }
        }
    } else {
        for (uint32_t mask = 1UL; aNumberOfBits > 0; mask <<= 1, aNumberOfBits--) {
            if (tRawBufPointer >= tRawBufEnd) return false;
            if      (matchMark(*tRawBufPointer, aOneMarkMicros))  tDecodedData |= mask;
            else if (!matchMark(*tRawBufPointer, aZeroMarkMicros)) return false;
            tRawBufPointer++;

            if (tRawBufPointer < tRawBufEnd) {
                if (!matchSpace(*tRawBufPointer, aBitSpaceMicros)) return false;
                tRawBufPointer++;
            }
        }
    }

    decodedIRData.decodedRawData = tDecodedData;
    return true;
}

/* -------------------------------------------------------------------------------------------------
 * Pulse-distance decoder
 * Bit value is encoded in the space duration (one space = long, zero space = short).
 * ------------------------------------------------------------------------------------------------- */
bool IRrecv::decodePulseDistanceData(uint8_t aNumberOfBits, uint8_t aStartOffset,
                                      uint16_t aBitMarkMicros, uint16_t aOneSpaceMicros,
                                      uint16_t aZeroSpaceMicros, bool aMSBfirst) {

    RAWBUF_DATA_TYPE *tRawBufPointer = &decodedIRData.rawDataPtr->rawbuf[aStartOffset];
    RAWBUF_DATA_TYPE *tRawBufEnd     = &decodedIRData.rawDataPtr->rawbuf[decodedIRData.rawDataPtr->rawlen];
    uint32_t tDecodedData = 0;

    if (aMSBfirst) {
        for (uint_fast8_t i = 0; i < aNumberOfBits; i++) {
            if (tRawBufPointer >= tRawBufEnd) return false;
            if (!matchMark(*tRawBufPointer, aBitMarkMicros)) return false;
            tRawBufPointer++;

            if (tRawBufPointer >= tRawBufEnd) return false;
            if      (matchSpace(*tRawBufPointer, aOneSpaceMicros))  tDecodedData = (tDecodedData << 1) | 1;
            else if (matchSpace(*tRawBufPointer, aZeroSpaceMicros)) tDecodedData = (tDecodedData << 1);
            else return false;
            tRawBufPointer++;
        }
    } else {
        for (uint32_t mask = 1UL; aNumberOfBits > 0; mask <<= 1, aNumberOfBits--) {
            if (tRawBufPointer >= tRawBufEnd) return false;
            if (!matchMark(*tRawBufPointer, aBitMarkMicros)) return false;
            tRawBufPointer++;

            if (tRawBufPointer >= tRawBufEnd) return false;
            if      (matchSpace(*tRawBufPointer, aOneSpaceMicros))  tDecodedData |= mask;
            else if (!matchSpace(*tRawBufPointer, aZeroSpaceMicros)) return false;
            tRawBufPointer++;
        }
    }

    decodedIRData.decodedRawData = tDecodedData;
    return true;
}

/* -------------------------------------------------------------------------------------------------
 * Biphase helpers (used by RC5 / RC6 decoders)
 * ------------------------------------------------------------------------------------------------- */
uint8_t  sBiphaseDecodeRawbuffOffset;
uint16_t sCurrentTimingIntervals;
uint8_t  sUsedTimingIntervals;
uint16_t sBiphaseTimeUnit;

void IRrecv::initBiphaselevel(uint8_t aRCDecodeRawbuffOffset, uint16_t aBiphaseTimeUnit) {
    sBiphaseDecodeRawbuffOffset = aRCDecodeRawbuffOffset;
    sBiphaseTimeUnit            = aBiphaseTimeUnit;
    sUsedTimingIntervals        = 0;
}

uint8_t IRrecv::getBiphaselevel() {
    if (sBiphaseDecodeRawbuffOffset >= decodedIRData.rawDataPtr->rawlen) return SPACE;

    uint8_t tLevel = (sBiphaseDecodeRawbuffOffset & 1);

    if (sUsedTimingIntervals == 0) {
        uint16_t tCurrent = decodedIRData.rawDataPtr->rawbuf[sBiphaseDecodeRawbuffOffset];
        int16_t  corr     = (tLevel == MARK) ? MARK_EXCESS_MICROS : -MARK_EXCESS_MICROS;

        if      (matchTicks(tCurrent,     sBiphaseTimeUnit + corr)) sCurrentTimingIntervals = 1;
        else if (matchTicks(tCurrent, 2 * sBiphaseTimeUnit + corr)) sCurrentTimingIntervals = 2;
        else if (matchTicks(tCurrent, 3 * sBiphaseTimeUnit + corr)) sCurrentTimingIntervals = 3;
        else return (uint8_t)-1;  // no match — signal a decode error to the caller
    }

    sUsedTimingIntervals++;
    if (sUsedTimingIntervals >= sCurrentTimingIntervals) {
        sUsedTimingIntervals = 0;
        sBiphaseDecodeRawbuffOffset++;
    }
    return tLevel;
}

/* -------------------------------------------------------------------------------------------------
 * Hash decoder  (FNV-1 32-bit, protocol-agnostic fingerprint)
 * ------------------------------------------------------------------------------------------------- */
#if defined(DECODE_HASH)
uint8_t IRrecv::compare(unsigned int oldval, unsigned int newval) {
    if (newval * 10 < oldval * 8) return 0;
    if (oldval * 10 < newval * 8) return 2;
    return 1;
}

// FNV-1 32-bit constants (https://en.wikipedia.org/wiki/Fowler–Noll–Vo_hash_function)
#define FNV_PRIME_32 16777619UL
#define FNV_BASIS_32 2166136261UL

bool IRrecv::decodeHash() {
    if (decodedIRData.rawDataPtr->rawlen < 6) return false;

    uint32_t hash = FNV_BASIS_32;  // BUG FIX: was declared as plain `long` (signed, 16-bit on AVR).
                                   // FNV_BASIS_32 (2166136261) exceeds INT16_MAX and INT32_MAX on AVR,
                                   // causing silent overflow. uint32_t is the correct type.

#if RAW_BUFFER_LENGTH <= 254
    uint8_t  i;
#else
    uint16_t i;
#endif
    for (i = 1; (i + 2) < decodedIRData.rawDataPtr->rawlen; i++) {
        uint8_t value = compare(decodedIRData.rawDataPtr->rawbuf[i],
                                decodedIRData.rawDataPtr->rawbuf[i + 2]);
        hash = (hash * FNV_PRIME_32) ^ value;
    }

    decodedIRData.decodedRawData = hash;
    decodedIRData.numberOfBits   = 32;
    decodedIRData.protocol       = UNKNOWN;
    return true;
}
#endif  // DECODE_HASH

/* -------------------------------------------------------------------------------------------------
 * Tick / microsecond matching functions
 * ------------------------------------------------------------------------------------------------- */
bool matchTicks(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros) {
    return (aMeasuredTicks >= TICKS_LOW(aMatchValueMicros) &&
            aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros));
}

// Legacy name aliases
bool MATCH(uint16_t measured_ticks, uint16_t desired_us) {
    return matchTicks(measured_ticks, desired_us);
}

bool matchMark(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros) {
    return (aMeasuredTicks >= TICKS_LOW(aMatchValueMicros + MARK_EXCESS_MICROS) &&
            aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros + MARK_EXCESS_MICROS));
}

bool MATCH_MARK(uint16_t measured_ticks, uint16_t desired_us) {
    return matchMark(measured_ticks, desired_us);
}

bool matchSpace(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros) {
    // Protect against unsigned underflow when MARK_EXCESS_MICROS > aMatchValueMicros
    if (aMatchValueMicros < MARK_EXCESS_MICROS) return false;  // BUG FIX: was silent underflow → wrong ticks
    return (aMeasuredTicks >= TICKS_LOW(aMatchValueMicros - MARK_EXCESS_MICROS) &&
            aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros - MARK_EXCESS_MICROS));
}

bool MATCH_SPACE(uint16_t measured_ticks, uint16_t desired_us) {
    return matchSpace(measured_ticks, desired_us);
}

int getMarkExcessMicros() {
    return MARK_EXCESS_MICROS;
}

/* -------------------------------------------------------------------------------------------------
 * MICROS_PER_TICK runtime accessor
 * Allows protocol decoders to query the active tick resolution without
 * hard-coding the macro value (important for multi-receiver configurations).
 * ------------------------------------------------------------------------------------------------- */
int getMICROS_PER_TICK() {
    return MICROS_PER_TICK;
}

/* -------------------------------------------------------------------------------------------------
 * MultiIR ISR helpers
 * Port registers are read once per interrupt (PINB/PINC/PIND on AVR) and
 * then shared across all receiver state machines to save cycles.
 * ------------------------------------------------------------------------------------------------- */
static inline uint8_t readInputLevelFor(irparams_struct &p,
                                        uint8_t pinb, uint8_t pinc, uint8_t pind) {
#if defined(__AVR__)
    if (p.IRReceivePinPortInputRegister == &PINB) return (pinb & p.IRReceivePinMask);
    if (p.IRReceivePinPortInputRegister == &PINC) return (pinc & p.IRReceivePinMask);
    return (pind & p.IRReceivePinMask);  // default: PIND
#else
    (void)pinb; (void)pinc; (void)pind;
    return (uint8_t)digitalRead(p.IRReceivePin);
#endif
}

static inline void ProcessOneIRParam(irparams_struct &p, uint8_t tIRInputLevel) {
    // Increment tick counter, saturate at 0xFFFF to prevent wraparound
    if (p.TickCounterForISR < 0xFFFF) p.TickCounterForISR++;

    switch (p.StateForISR) {

    case IR_REC_STATE_IDLE:
        if (tIRInputLevel == INPUT_MARK) {
            if (p.TickCounterForISR > RECORD_GAP_TICKS) {
                // Valid inter-command gap seen — start recording a new frame
                p.OverflowFlag  = false;
                p.rawbuf[0]     = clampTicksToRawbuf(p.TickCounterForISR);
                p.rawlen        = 1;
                p.StateForISR   = IR_REC_STATE_MARK;
            }
            p.TickCounterForISR = 0;
        }
        break;

    case IR_REC_STATE_MARK:
      if (tIRInputLevel != INPUT_MARK) {
        if (p.TickCounterForISR < MIN_SIGNAL_TICKS) {
            p.TickCounterForISR = 0;
            break;
        }
        if (p.rawlen >= RAW_BUFFER_LENGTH) {
            // Overflow from noise — go straight back to IDLE
            // instead of STOP so the receiver self-recovers
            // without needing resume() from the main loop.
            p.OverflowFlag      = true;
            p.rawlen            = 0;
            p.StateForISR       = IR_REC_STATE_IDLE;  // NOT STOP
        } else {
            p.rawbuf[p.rawlen++] = clampTicksToRawbuf(p.TickCounterForISR);
            p.StateForISR        = IR_REC_STATE_SPACE;
        }
        p.TickCounterForISR = 0;
    }
    break;

case IR_REC_STATE_SPACE:
    if (tIRInputLevel == INPUT_MARK) {
        if (p.TickCounterForISR < MIN_SIGNAL_TICKS) {
            p.TickCounterForISR = 0;
            break;
        }
        if (p.rawlen >= RAW_BUFFER_LENGTH) {
            // Same — self-recover to IDLE on overflow
            p.OverflowFlag      = true;
            p.rawlen            = 0;
            p.StateForISR       = IR_REC_STATE_IDLE;  // NOT STOP
        } else {
            p.rawbuf[p.rawlen++] = clampTicksToRawbuf(p.TickCounterForISR);
            p.StateForISR        = IR_REC_STATE_MARK;
        }
        p.TickCounterForISR = 0;
    } else if (p.TickCounterForISR > RECORD_GAP_TICKS) {
        p.StateForISR = IR_REC_STATE_STOP;  // legitimate frame end — keep STOP
    }
    break;

    case IR_REC_STATE_STOP:
        // Waiting for resume(); swallow any spurious marks
        if (tIRInputLevel == INPUT_MARK) {
            p.TickCounterForISR = 0;
        }
        break;

    default:
        // Should never happen — reset to safe state
        p.StateForISR = IR_REC_STATE_IDLE;
        break;
    }
}

/* -------------------------------------------------------------------------------------------------
 * Timer ISR — fired once per MICROS_PER_TICK by the hardware timer
 * ------------------------------------------------------------------------------------------------- */
#if defined(TIMER_INTR_NAME)
ISR(TIMER_INTR_NAME)
#else
ISR()
#endif
{
    TIMER_RESET_INTR_PENDING;  // acknowledge the interrupt before any work

#if defined(__AVR__)
    // Snapshot all port registers once to minimise time inside the ISR
    const uint8_t pinb = PINB;
    const uint8_t pinc = PINC;
    const uint8_t pind = PIND;
#else
    const uint8_t pinb = 0, pinc = 0, pind = 0;
#endif

    const int n = IrParamsList().GetCount();
    for (int i = 0; i < n; ++i) {
        irparams_struct *pp = IrParamsList().GetItem(i);
        if (!pp) continue;  // defensive: skip any null entries
        ProcessOneIRParam(*pp, readInputLevelFor(*pp, pinb, pinc, pind));
    }
}

/** @} */

#endif // IR_RECEIVE_HPP
