
#ifndef IR_RECEIVE_HPP
#define IR_RECEIVE_HPP

#include <Arduino.h>
#include "IRremoteInt.h"
#include "CppList.hpp"

/** \addtogroup Receiving Receiving IR data for multiple protocols
 * @{
 */

/* -------------------------------------------------------------------------------------------------
 * MultiIR registry: function-static singleton to avoid static initialization order problems
 * ------------------------------------------------------------------------------------------------- */
static inline CppList& IrParamsList() {
    static CppList list;
    return list;
}

/* -------------------------------------------------------------------------------------------------
 * Global receiver instance (kept for compatibility with IRremote usage patterns)
 * ------------------------------------------------------------------------------------------------- */
IRrecv IrReceiver;

/* -------------------------------------------------------------------------------------------------
 * Shared timer enable refcount (because the timer ISR is global on ATmega328P)
 * ------------------------------------------------------------------------------------------------- */
static volatile uint8_t gActiveReceivers = 0;

/* -------------------------------------------------------------------------------------------------
 * RAWBUF_DATA_TYPE safe storage:
 * If RAWBUF_DATA_TYPE is uint8_t, clamp tick values to 0xFF to avoid wraparound.
 * ------------------------------------------------------------------------------------------------- */
static inline RAWBUF_DATA_TYPE clampTicksToRawbuf(uint16_t ticks) {
    if (sizeof(RAWBUF_DATA_TYPE) == 1) {
        return (ticks > 0xFF) ? (RAWBUF_DATA_TYPE)0xFF : (RAWBUF_DATA_TYPE)ticks;
    }
    return (RAWBUF_DATA_TYPE)ticks;
}

/* -------------------------------------------------------------------------------------------------
 * IRrecv constructors
 * ------------------------------------------------------------------------------------------------- */
IRrecv::IRrecv() {
    decodedIRData.rawDataPtr = &irparams; // per-instance state
    setReceivePin(0);

#if !defined(DISABLE_LED_FEEDBACK_FOR_RECEIVE)
    setLEDFeedback(0, false);
#endif

    // Register this receiver state in the global list (safe if called multiple times)
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
 * Stream like API
 * ------------------------------------------------------------------------------------------------- */
void IRrecv::begin(uint8_t aReceivePin, bool aEnableLEDFeedback, uint8_t aFeedbackLEDPin) {
    setReceivePin(aReceivePin);
#if !defined(DISABLE_LED_FEEDBACK_FOR_RECEIVE)
    setLEDFeedback(aFeedbackLEDPin, aEnableLEDFeedback);
#else
    (void)aEnableLEDFeedback;
    (void)aFeedbackLEDPin;
#endif

    // Register (safe if already registered)
    noInterrupts();
    IrParamsList().Add(&irparams);
    interrupts();

    enableIRIn();
}

void IRrecv::setReceivePin(uint8_t aReceivePinNumber) {
    irparams.IRReceivePin = aReceivePinNumber;

#if defined(__AVR__)
    irparams.IRReceivePinMask = digitalPinToBitMask(aReceivePinNumber);
    irparams.IRReceivePinPortInputRegister = portInputRegister(digitalPinToPort(aReceivePinNumber));
#endif
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
 * enableIRIn / disableIRIn (reference-count timer ISR)
 * ------------------------------------------------------------------------------------------------- */
void IRrecv::enableIRIn() {
    noInterrupts();

    // Configure & enable the timer ISR only when the first receiver is enabled.
    if (gActiveReceivers++ == 0) {
        timerConfigForReceive();
        TIMER_ENABLE_RECEIVE_INTR;
        TIMER_RESET_INTR_PENDING;
    }

    // Reset only this receiver’s state machine
    irparams.StateForISR = IR_REC_STATE_IDLE;
    irparams.TickCounterForISR = 0;
    irparams.OverflowFlag = false;
    irparams.rawlen = 0;

    interrupts();

    pinMode(irparams.IRReceivePin, INPUT);
}

void IRrecv::disableIRIn() {
    noInterrupts();
    if (gActiveReceivers > 0 && --gActiveReceivers == 0) {
        TIMER_DISABLE_RECEIVE_INTR;
    }
    interrupts();
}

bool IRrecv::isIdle() {
    return (irparams.StateForISR == IR_REC_STATE_IDLE || irparams.StateForISR == IR_REC_STATE_STOP);
}

void IRrecv::resume() {
    if (irparams.StateForISR == IR_REC_STATE_STOP) {
        irparams.StateForISR = IR_REC_STATE_IDLE;
    }
}

void IRrecv::initDecodedIRData() {
    if (irparams.OverflowFlag) {
        irparams.OverflowFlag = false;
        irparams.rawlen = 0;
        decodedIRData.flags = IRDATA_FLAGS_WAS_OVERFLOW;
        DEBUG_PRINTLN("Overflow happened");
    } else {
        decodedIRData.flags = IRDATA_FLAGS_EMPTY;
        lastDecodedProtocol = decodedIRData.protocol;
        lastDecodedCommand  = decodedIRData.command;
        lastDecodedAddress  = decodedIRData.address;
    }

    decodedIRData.protocol = UNKNOWN;
    decodedIRData.command = 0;
    decodedIRData.address = 0;
    decodedIRData.decodedRawData = 0;
    decodedIRData.numberOfBits = 0;
}

bool IRrecv::available() {
    return (irparams.StateForISR == IR_REC_STATE_STOP);
}

IRData* IRrecv::read() {
    if (irparams.StateForISR != IR_REC_STATE_STOP) return NULL;
    if (decode()) return &decodedIRData;
    return NULL;
}

/* -------------------------------------------------------------------------------------------------
 * decode() (same order as library defaults)
 * ------------------------------------------------------------------------------------------------- */
bool IRrecv::decode() {
    if (irparams.StateForISR != IR_REC_STATE_STOP) return false;

    initDecodedIRData();

    if (decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
        decodedIRData.protocol = UNKNOWN;
        return true; // data available (overflow)
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

    return true; // frame captured but unknown protocol
}

/* -------------------------------------------------------------------------------------------------
 * Common decode functions (RAWBUF_DATA_TYPE pointers)
 * ------------------------------------------------------------------------------------------------- */
bool IRrecv::decodePulseWidthData(uint8_t aNumberOfBits, uint8_t aStartOffset,
        uint16_t aOneMarkMicros, uint16_t aZeroMarkMicros,
        uint16_t aBitSpaceMicros, bool aMSBfirst) {

    RAWBUF_DATA_TYPE *tRawBufPointer = &decodedIRData.rawDataPtr->rawbuf[aStartOffset];
    uint32_t tDecodedData = 0;

    if (aMSBfirst) {
        for (uint_fast8_t i = 0; i < aNumberOfBits; i++) {
            if (matchMark(*tRawBufPointer, aOneMarkMicros)) tDecodedData = (tDecodedData << 1) | 1;
            else if (matchMark(*tRawBufPointer, aZeroMarkMicros)) tDecodedData = (tDecodedData << 1);
            else return false;
            tRawBufPointer++;

            if (tRawBufPointer < &decodedIRData.rawDataPtr->rawbuf[decodedIRData.rawDataPtr->rawlen]) {
                if (!matchSpace(*tRawBufPointer, aBitSpaceMicros)) return false;
                tRawBufPointer++;
            }
        }
    } else {
        for (uint32_t mask = 1UL; aNumberOfBits > 0; mask <<= 1, aNumberOfBits--) {
            if (matchMark(*tRawBufPointer, aOneMarkMicros)) tDecodedData |= mask;
            else if (!matchMark(*tRawBufPointer, aZeroMarkMicros)) return false;
            tRawBufPointer++;

            if (tRawBufPointer < &decodedIRData.rawDataPtr->rawbuf[decodedIRData.rawDataPtr->rawlen]) {
                if (!matchSpace(*tRawBufPointer, aBitSpaceMicros)) return false;
                tRawBufPointer++;
            }
        }
    }

    decodedIRData.decodedRawData = tDecodedData;
    return true;
}

bool IRrecv::decodePulseDistanceData(uint8_t aNumberOfBits, uint8_t aStartOffset,
        uint16_t aBitMarkMicros, uint16_t aOneSpaceMicros,
        uint16_t aZeroSpaceMicros, bool aMSBfirst) {

    RAWBUF_DATA_TYPE *tRawBufPointer = &decodedIRData.rawDataPtr->rawbuf[aStartOffset];
    uint32_t tDecodedData = 0;

    if (aMSBfirst) {
        for (uint_fast8_t i = 0; i < aNumberOfBits; i++) {
            if (!matchMark(*tRawBufPointer, aBitMarkMicros)) return false;
            tRawBufPointer++;

            if (matchSpace(*tRawBufPointer, aOneSpaceMicros)) tDecodedData = (tDecodedData << 1) | 1;
            else if (matchSpace(*tRawBufPointer, aZeroSpaceMicros)) tDecodedData = (tDecodedData << 1);
            else return false;
            tRawBufPointer++;
        }
    } else {
        for (uint32_t mask = 1UL; aNumberOfBits > 0; mask <<= 1, aNumberOfBits--) {
            if (!matchMark(*tRawBufPointer, aBitMarkMicros)) return false;
            tRawBufPointer++;

            if (matchSpace(*tRawBufPointer, aOneSpaceMicros)) tDecodedData |= mask;
            else if (!matchSpace(*tRawBufPointer, aZeroSpaceMicros)) return false;
            tRawBufPointer++;
        }
    }

    decodedIRData.decodedRawData = tDecodedData;
    return true;
}

/* -------------------------------------------------------------------------------------------------
 * Biphase helpers (RC5/RC6)
 * ------------------------------------------------------------------------------------------------- */
uint8_t sBiphaseDecodeRawbuffOffset;
uint16_t sCurrentTimingIntervals;
uint8_t sUsedTimingIntervals;
uint16_t sBiphaseTimeUnit;

void IRrecv::initBiphaselevel(uint8_t aRCDecodeRawbuffOffset, uint16_t aBiphaseTimeUnit) {
    sBiphaseDecodeRawbuffOffset = aRCDecodeRawbuffOffset;
    sBiphaseTimeUnit = aBiphaseTimeUnit;
    sUsedTimingIntervals = 0;
}

uint8_t IRrecv::getBiphaselevel() {
    if (sBiphaseDecodeRawbuffOffset >= decodedIRData.rawDataPtr->rawlen) return SPACE;
    uint8_t tLevel = (sBiphaseDecodeRawbuffOffset & 1);

    if (sUsedTimingIntervals == 0) {
        uint16_t tCurrent = decodedIRData.rawDataPtr->rawbuf[sBiphaseDecodeRawbuffOffset];
        int16_t corr = (tLevel == MARK) ? MARK_EXCESS_MICROS : -MARK_EXCESS_MICROS;

        if (matchTicks(tCurrent, (sBiphaseTimeUnit) + corr)) sCurrentTimingIntervals = 1;
        else if (matchTicks(tCurrent, (2 * sBiphaseTimeUnit) + corr)) sCurrentTimingIntervals = 2;
        else if (matchTicks(tCurrent, (3 * sBiphaseTimeUnit) + corr)) sCurrentTimingIntervals = 3;
        else return (uint8_t)-1;
    }

    sUsedTimingIntervals++;
    if (sUsedTimingIntervals >= sCurrentTimingIntervals) {
        sUsedTimingIntervals = 0;
        sBiphaseDecodeRawbuffOffset++;
    }
    return tLevel;
}

/* -------------------------------------------------------------------------------------------------
 * Hash decoder helpers (kept)
 * ------------------------------------------------------------------------------------------------- */
#if defined(DECODE_HASH)
uint8_t IRrecv::compare(unsigned int oldval, unsigned int newval) {
    if (newval * 10 < oldval * 8) return 0;
    if (oldval * 10 < newval * 8) return 2;
    return 1;
}

#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

bool IRrecv::decodeHash() {
    long hash = FNV_BASIS_32;

    if (decodedIRData.rawDataPtr->rawlen < 6) return false;

#if RAW_BUFFER_LENGTH <= 254
    uint8_t i;
#else
    uint16_t i;
#endif
    for (i = 1; (i + 2) < decodedIRData.rawDataPtr->rawlen; i++) {
        uint8_t value = compare(decodedIRData.rawDataPtr->rawbuf[i], decodedIRData.rawDataPtr->rawbuf[i + 2]);
        hash = (hash * FNV_PRIME_32) ^ value;
    }

    decodedIRData.decodedRawData = hash;
    decodedIRData.numberOfBits = 32;
    decodedIRData.protocol = UNKNOWN;
    return true;
}
#endif

/* -------------------------------------------------------------------------------------------------
 * Match functions (use ticks)
 * ------------------------------------------------------------------------------------------------- */
bool matchTicks(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros) {
    return ((aMeasuredTicks >= TICKS_LOW(aMatchValueMicros)) && (aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros)));
}

bool MATCH(uint16_t measured_ticks, uint16_t desired_us) { return matchTicks(measured_ticks, desired_us); }

bool matchMark(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros) {
    return ((aMeasuredTicks >= TICKS_LOW(aMatchValueMicros + MARK_EXCESS_MICROS)) &&
            (aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros + MARK_EXCESS_MICROS)));
}

bool MATCH_MARK(uint16_t measured_ticks, uint16_t desired_us) { return matchMark(measured_ticks, desired_us); }

bool matchSpace(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros) {
    return ((aMeasuredTicks >= TICKS_LOW(aMatchValueMicros - MARK_EXCESS_MICROS)) &&
            (aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros - MARK_EXCESS_MICROS)));
}

bool MATCH_SPACE(uint16_t measured_ticks, uint16_t desired_us) { return matchSpace(measured_ticks, desired_us); }

int getMarkExcessMicros() { return MARK_EXCESS_MICROS; }

/* -------------------------------------------------------------------------------------------------
 * MICROS_PER_TICK getter used by your modified protocol decoders
 * ------------------------------------------------------------------------------------------------- */
int getMICROS_PER_TICK() {
    return MICROS_PER_TICK;
}

/* -------------------------------------------------------------------------------------------------
 * MultiIR ISR: process each registered receiver
 * - reads PINB/PINC/PIND once per interrupt for speed
 * ------------------------------------------------------------------------------------------------- */

static inline uint8_t readInputLevelFor(irparams_struct &p, uint8_t pinb, uint8_t pinc, uint8_t pind) {
#if defined(__AVR__)
    // Determine which cached port to use by comparing register pointer
    if (p.IRReceivePinPortInputRegister == &PINB) return (pinb & p.IRReceivePinMask);
    if (p.IRReceivePinPortInputRegister == &PINC) return (pinc & p.IRReceivePinMask);
    return (pind & p.IRReceivePinMask);
#else
    return (uint8_t)digitalRead(p.IRReceivePin);
#endif
}

static inline void ProcessOneIRParam(irparams_struct &p, uint8_t tIRInputLevel) {
    // Increase tick counter (clip to 0xFFFF)
    if (p.TickCounterForISR < 0xFFFF) p.TickCounterForISR++;

    if (p.StateForISR == IR_REC_STATE_IDLE) {
        if (tIRInputLevel == INPUT_MARK) {
            if (p.TickCounterForISR > RECORD_GAP_TICKS) {
                p.OverflowFlag = false;
                p.rawbuf[0] = clampTicksToRawbuf(p.TickCounterForISR);
                p.rawlen = 1;
                p.StateForISR = IR_REC_STATE_MARK;
            }
            p.TickCounterForISR = 0;
        }

    } else if (p.StateForISR == IR_REC_STATE_MARK) {
        if (tIRInputLevel != INPUT_MARK) {
            if (p.rawlen >= RAW_BUFFER_LENGTH) {
                p.OverflowFlag = true;
                p.StateForISR = IR_REC_STATE_STOP;
            } else {
                p.rawbuf[p.rawlen++] = clampTicksToRawbuf(p.TickCounterForISR);
                p.StateForISR = IR_REC_STATE_SPACE;
            }
            p.TickCounterForISR = 0;
        }

    } else if (p.StateForISR == IR_REC_STATE_SPACE) {
        if (tIRInputLevel == INPUT_MARK) {
            if (p.rawlen >= RAW_BUFFER_LENGTH) {
                p.OverflowFlag = true;
                p.StateForISR = IR_REC_STATE_STOP;
            } else {
                p.rawbuf[p.rawlen++] = clampTicksToRawbuf(p.TickCounterForISR);
                p.StateForISR = IR_REC_STATE_MARK;
            }
            p.TickCounterForISR = 0;

        } else if (p.TickCounterForISR > RECORD_GAP_TICKS) {
            p.StateForISR = IR_REC_STATE_STOP;
        }

    } else { // STOP
        if (tIRInputLevel == INPUT_MARK) {
            p.TickCounterForISR = 0;
        }
    }
}

#if defined(TIMER_INTR_NAME)
ISR(TIMER_INTR_NAME)
#else
ISR()
#endif
{
    // Reset interrupt pending ONCE per timer tick
    TIMER_RESET_INTR_PENDING;

#if defined(__AVR__)
    uint8_t pinb = PINB;
    uint8_t pinc = PINC;
    uint8_t pind = PIND;
#else
    uint8_t pinb = 0, pinc = 0, pind = 0;
#endif

    int n = IrParamsList().GetCount();
    for (int i = 0; i < n; ++i) {
        irparams_struct *pp = IrParamsList().GetItem(i);
        if (!pp) continue;
        uint8_t level = readInputLevelFor(*pp, pinb, pinc, pind);
        ProcessOneIRParam(*pp, level);
    }
}

#endif // IR_RECEIVE_HPP
#pragma once
