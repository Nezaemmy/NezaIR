#pragma once
#ifndef IR_RECEIVE_HPP
#define IR_RECEIVE_HPP

#include <Arduino.h>
#include "IRremoteInt.h"
#include "CppList.hpp"

#ifndef IR_INVALID_PIN
#define IR_INVALID_PIN 0xFF
#endif

static inline CppList &IrParamsList() {
    static CppList list;
    return list;
}

static volatile uint8_t gActiveReceivers = 0;

static inline RAWBUF_DATA_TYPE clampTicksToRawbuf(uint16_t ticks) {
    if (sizeof(RAWBUF_DATA_TYPE) == 1 && ticks > 0xFF) {
        return (RAWBUF_DATA_TYPE)0xFF;
    }
    return (RAWBUF_DATA_TYPE)ticks;
}

IRrecv::IRrecv() {
    decodedIRData.rawDataPtr = &irparams;

    irparams.StateForISR       = IR_REC_STATE_IDLE;
    irparams.IRReceivePin      = IR_INVALID_PIN;
    irparams.rawlen            = 0;
    irparams.TickCounterForISR = 0;
    irparams.OverflowFlag      = false;

#if defined(__AVR__)
    irparams.IRReceivePinMask = 0;
    irparams.IRReceivePinPortInputRegister = nullptr;
#endif

    decodedIRData.protocol       = UNKNOWN;
    decodedIRData.address        = 0;
    decodedIRData.command        = 0;
    decodedIRData.extra          = 0;
    decodedIRData.numberOfBits   = 0;
    decodedIRData.flags          = IRDATA_FLAGS_EMPTY;
    decodedIRData.decodedRawData = 0;

    lastDecodedProtocol = UNKNOWN;
    lastDecodedAddress  = 0;
    lastDecodedCommand  = 0;
    repeatCount         = 0;
    isActive            = false;

#if !defined(DISABLE_LED_FEEDBACK_FOR_RECEIVE)
    setLEDFeedback(0, false);
#endif

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

IRrecv::~IRrecv() {
    end();
}

void IRrecv::begin(uint8_t aReceivePin, bool aEnableLEDFeedback, uint8_t aFeedbackLEDPin) {
    setReceivePin(aReceivePin);

#if !defined(DISABLE_LED_FEEDBACK_FOR_RECEIVE)
    setLEDFeedback(aFeedbackLEDPin, aEnableLEDFeedback);
#else
    (void)aEnableLEDFeedback;
    (void)aFeedbackLEDPin;
#endif

    noInterrupts();
    IrParamsList().Add(&irparams);
    interrupts();

    enableIRIn();
}

void IRrecv::setReceivePin(uint8_t aReceivePinNumber) {
    if (aReceivePinNumber == IR_INVALID_PIN) return;

    irparams.IRReceivePin = aReceivePinNumber;

#if defined(__AVR__)
    irparams.IRReceivePinMask = digitalPinToBitMask(aReceivePinNumber);
    irparams.IRReceivePinPortInputRegister = portInputRegister(digitalPinToPort(aReceivePinNumber));
#endif

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

    noInterrupts();
    IrParamsList().Remove(&irparams);
    interrupts();
}

void IRrecv::enableIRIn() {
    if (irparams.IRReceivePin == IR_INVALID_PIN) return;

    noInterrupts();

    if (!isActive) {
        isActive = true;

        if (gActiveReceivers++ == 0) {
            timerConfigForReceive();
            TIMER_RESET_INTR_PENDING;
            TIMER_ENABLE_RECEIVE_INTR;
        }
    }

    irparams.StateForISR       = IR_REC_STATE_IDLE;
    irparams.TickCounterForISR = 0;
    irparams.OverflowFlag      = false;
    irparams.rawlen            = 0;

    interrupts();

    pinMode(irparams.IRReceivePin, INPUT_PULLUP);
}

void IRrecv::disableIRIn() {
    noInterrupts();

    if (isActive) {
        isActive = false;

        if (gActiveReceivers > 0 && --gActiveReceivers == 0) {
            TIMER_DISABLE_RECEIVE_INTR;
        }
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

void IRrecv::initDecodedIRData() {
    if (irparams.OverflowFlag) {
        irparams.OverflowFlag = false;
        irparams.rawlen       = 0;
        decodedIRData.flags   = IRDATA_FLAGS_WAS_OVERFLOW;
        DEBUG_PRINTLN(F("Overflow happened"));
    } else {
        decodedIRData.flags    = IRDATA_FLAGS_EMPTY;
        lastDecodedProtocol    = decodedIRData.protocol;
        lastDecodedCommand     = decodedIRData.command;
        lastDecodedAddress     = decodedIRData.address;
    }

    decodedIRData.protocol       = UNKNOWN;
    decodedIRData.command        = 0;
    decodedIRData.address        = 0;
    decodedIRData.extra          = 0;
    decodedIRData.decodedRawData = 0;
    decodedIRData.numberOfBits   = 0;
}

bool IRrecv::available() {
    return (irparams.StateForISR == IR_REC_STATE_STOP);
}

IRData *IRrecv::read() {
    if (irparams.StateForISR != IR_REC_STATE_STOP) return nullptr;
    if (decode()) return &decodedIRData;
    return nullptr;
}

bool IRrecv::decode() {
    if (irparams.StateForISR != IR_REC_STATE_STOP) return false;

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

bool IRrecv::decodePulseWidthData(uint8_t aNumberOfBits, uint8_t aStartOffset,
                                  uint16_t aOneMarkMicros, uint16_t aZeroMarkMicros,
                                  uint16_t aBitSpaceMicros, bool aMSBfirst) {
    if (aNumberOfBits > 32) return false;

    RAWBUF_DATA_TYPE *tRawBufPointer = &decodedIRData.rawDataPtr->rawbuf[aStartOffset];
    RAWBUF_DATA_TYPE *tRawBufEnd     = &decodedIRData.rawDataPtr->rawbuf[decodedIRData.rawDataPtr->rawlen];

    uint32_t tDecodedData = 0;

    if (aMSBfirst) {
        for (uint_fast8_t i = 0; i < aNumberOfBits; i++) {
            if (tRawBufPointer >= tRawBufEnd) return false;

            if (matchMark(*tRawBufPointer, aOneMarkMicros)) {
                tDecodedData = (tDecodedData << 1) | 1;
            } else if (matchMark(*tRawBufPointer, aZeroMarkMicros)) {
                tDecodedData = (tDecodedData << 1);
            } else {
                return false;
            }

            tRawBufPointer++;

            if (tRawBufPointer < tRawBufEnd) {
                if (!matchSpace(*tRawBufPointer, aBitSpaceMicros)) return false;
                tRawBufPointer++;
            }
        }
    } else {
        for (uint32_t mask = 1UL; aNumberOfBits > 0; mask <<= 1, aNumberOfBits--) {
            if (tRawBufPointer >= tRawBufEnd) return false;

            if (matchMark(*tRawBufPointer, aOneMarkMicros)) {
                tDecodedData |= mask;
            } else if (!matchMark(*tRawBufPointer, aZeroMarkMicros)) {
                return false;
            }

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

bool IRrecv::decodePulseDistanceData(uint8_t aNumberOfBits, uint8_t aStartOffset,
                                     uint16_t aBitMarkMicros, uint16_t aOneSpaceMicros,
                                     uint16_t aZeroSpaceMicros, bool aMSBfirst) {
    if (aNumberOfBits > 32) return false;

    RAWBUF_DATA_TYPE *tRawBufPointer = &decodedIRData.rawDataPtr->rawbuf[aStartOffset];
    RAWBUF_DATA_TYPE *tRawBufEnd     = &decodedIRData.rawDataPtr->rawbuf[decodedIRData.rawDataPtr->rawlen];

    uint32_t tDecodedData = 0;

    if (aMSBfirst) {
        for (uint_fast8_t i = 0; i < aNumberOfBits; i++) {
            if (tRawBufPointer >= tRawBufEnd) return false;
            if (!matchMark(*tRawBufPointer, aBitMarkMicros)) return false;
            tRawBufPointer++;

            if (tRawBufPointer >= tRawBufEnd) return false;

            if (matchSpace(*tRawBufPointer, aOneSpaceMicros)) {
                tDecodedData = (tDecodedData << 1) | 1;
            } else if (matchSpace(*tRawBufPointer, aZeroSpaceMicros)) {
                tDecodedData = (tDecodedData << 1);
            } else {
                return false;
            }

            tRawBufPointer++;
        }
    } else {
        for (uint32_t mask = 1UL; aNumberOfBits > 0; mask <<= 1, aNumberOfBits--) {
            if (tRawBufPointer >= tRawBufEnd) return false;
            if (!matchMark(*tRawBufPointer, aBitMarkMicros)) return false;
            tRawBufPointer++;

            if (tRawBufPointer >= tRawBufEnd) return false;

            if (matchSpace(*tRawBufPointer, aOneSpaceMicros)) {
                tDecodedData |= mask;
            } else if (!matchSpace(*tRawBufPointer, aZeroSpaceMicros)) {
                return false;
            }

            tRawBufPointer++;
        }
    }

    decodedIRData.decodedRawData = tDecodedData;
    return true;
}

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
        int16_t corr = (tLevel == MARK) ? MARK_EXCESS_MICROS : -MARK_EXCESS_MICROS;

        if (matchTicks(tCurrent, sBiphaseTimeUnit + corr)) {
            sCurrentTimingIntervals = 1;
        } else if (matchTicks(tCurrent, 2 * sBiphaseTimeUnit + corr)) {
            sCurrentTimingIntervals = 2;
        } else if (matchTicks(tCurrent, 3 * sBiphaseTimeUnit + corr)) {
            sCurrentTimingIntervals = 3;
        } else {
            return (uint8_t)-1;
        }
    }

    sUsedTimingIntervals++;

    if (sUsedTimingIntervals >= sCurrentTimingIntervals) {
        sUsedTimingIntervals = 0;
        sBiphaseDecodeRawbuffOffset++;
    }

    return tLevel;
}

#if defined(DECODE_HASH)

uint8_t IRrecv::compare(unsigned int oldval, unsigned int newval) {
    if (newval * 10 < oldval * 8) return 0;
    if (oldval * 10 < newval * 8) return 2;
    return 1;
}

#define FNV_PRIME_32 16777619UL
#define FNV_BASIS_32 2166136261UL

bool IRrecv::decodeHash() {
    if (decodedIRData.rawDataPtr->rawlen < 6) return false;

    uint32_t hash = FNV_BASIS_32;

#if RAW_BUFFER_LENGTH <= 254
    uint8_t i;
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

#endif // DECODE_HASH

bool matchTicks(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros) {
    return (aMeasuredTicks >= TICKS_LOW(aMatchValueMicros) &&
            aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros));
}

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
    if (aMatchValueMicros < MARK_EXCESS_MICROS) return false;

    return (aMeasuredTicks >= TICKS_LOW(aMatchValueMicros - MARK_EXCESS_MICROS) &&
            aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros - MARK_EXCESS_MICROS));
}

bool MATCH_SPACE(uint16_t measured_ticks, uint16_t desired_us) {
    return matchSpace(measured_ticks, desired_us);
}

int getMarkExcessMicros() {
    return MARK_EXCESS_MICROS;
}

int getMICROS_PER_TICK() {
    return MICROS_PER_TICK;
}

static inline uint8_t readInputLevelFor(irparams_struct &p,
                                        uint8_t pinb,
                                        uint8_t pinc,
                                        uint8_t pind) {
#if defined(__AVR__)
    if (p.IRReceivePinPortInputRegister == &PINB) return (pinb & p.IRReceivePinMask);
    if (p.IRReceivePinPortInputRegister == &PINC) return (pinc & p.IRReceivePinMask);
    return (pind & p.IRReceivePinMask);
#else
    (void)pinb;
    (void)pinc;
    (void)pind;
    return (uint8_t)digitalRead(p.IRReceivePin);
#endif
}

static inline void ProcessOneIRParam(irparams_struct &p, uint8_t tIRInputLevel) {
    if (p.TickCounterForISR < 0xFFFF) {
        p.TickCounterForISR++;
    }

    switch (p.StateForISR) {
    case IR_REC_STATE_IDLE:
        if (tIRInputLevel == INPUT_MARK) {
            if (p.TickCounterForISR > RECORD_GAP_TICKS) {
                p.OverflowFlag = false;
                p.rawbuf[0]    = clampTicksToRawbuf(p.TickCounterForISR);
                p.rawlen       = 1;
                p.StateForISR  = IR_REC_STATE_MARK;
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
                p.OverflowFlag = true;
                p.rawlen       = 0;
                p.StateForISR  = IR_REC_STATE_STOP;
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
                p.OverflowFlag = true;
                p.rawlen       = 0;
                p.StateForISR  = IR_REC_STATE_STOP;
            } else {
                p.rawbuf[p.rawlen++] = clampTicksToRawbuf(p.TickCounterForISR);
                p.StateForISR        = IR_REC_STATE_MARK;
            }

            p.TickCounterForISR = 0;
        } else if (p.TickCounterForISR > RECORD_GAP_TICKS) {
            p.StateForISR = IR_REC_STATE_STOP;
        }
        break;

    case IR_REC_STATE_STOP:
        if (tIRInputLevel == INPUT_MARK) {
            p.TickCounterForISR = 0;
        }
        break;

    default:
        p.StateForISR = IR_REC_STATE_IDLE;
        break;
    }
}

#if defined(TIMER_INTR_NAME)
ISR(TIMER_INTR_NAME)
#else
ISR()
#endif
{
    TIMER_RESET_INTR_PENDING;

#if defined(__AVR__)
    const uint8_t pinb = PINB;
    const uint8_t pinc = PINC;
    const uint8_t pind = PIND;
#else
    const uint8_t pinb = 0;
    const uint8_t pinc = 0;
    const uint8_t pind = 0;
#endif

    const int n = IrParamsList().GetCount();

    for (int i = 0; i < n; ++i) {
        irparams_struct *pp = IrParamsList().GetItem(i);
        if (!pp) continue;
        if (pp->IRReceivePin == IR_INVALID_PIN) continue;
        if (pp->StateForISR == IR_REC_STATE_STOP) continue;

        ProcessOneIRParam(*pp, readInputLevelFor(*pp, pinb, pinc, pind));
    }
}

#endif // IR_RECEIVE_HPP
