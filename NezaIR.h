#pragma once
#ifndef NEZAIR_H
#define NEZAIR_H

// -------------------- Version --------------------
#define VERSION_NEZAIR "2.1.5"
#define VERSION_NEZAIR_MAJOR 2
#define VERSION_NEZAIR_MINOR 1
#define VERSION_NEZAIR_PATCH 5

// -------------------- Multi-IR defaults --------------------
#ifndef IR_MAXCOUNTIR
#define IR_MAXCOUNTIR 8
#endif

#ifndef MICROS_PER_TICK
#define MICROS_PER_TICK 150
#endif

#ifndef RAW_BUFFER_LENGTH
#define RAW_BUFFER_LENGTH 100
#endif

// -------------------- Protocol selection defaults --------------------
#if (!(defined(DECODE_DENON)      || \
       defined(DECODE_JVC)        || \
       defined(DECODE_KASEIKYO)   || \
       defined(DECODE_PANASONIC)  || \
       defined(DECODE_LG)         || \
       defined(DECODE_NEC)        || \
       defined(DECODE_SAMSUNG)    || \
       defined(DECODE_SONY)       || \
       defined(DECODE_RC5)        || \
       defined(DECODE_RC6)        || \
       defined(DECODE_HASH)       || \
       defined(DECODE_DISTANCE)   || \
       defined(DECODE_BOSEWAVE)   || \
       defined(DECODE_LEGO_PF)    || \
       defined(DECODE_MAGIQUEST)  || \
       defined(DECODE_WHYNTER)))

#define DECODE_NEC
#define DECODE_SONY
#define DECODE_SAMSUNG
#define DECODE_RC5
#define DECODE_RC6

#ifndef EXCLUDE_UNIVERSAL_PROTOCOLS
#define DECODE_DISTANCE
#define DECODE_HASH
#endif

#endif // default protocol selection

// -------------------- Receiver tuning --------------------
#ifndef MARK_EXCESS_MICROS
#define MARK_EXCESS_MICROS 20
#endif

#ifndef MIN_SIGNAL_TICKS
#define MIN_SIGNAL_TICKS 2
#endif

#ifndef RECORD_GAP_MICROS
#define RECORD_GAP_MICROS 5000
#endif

#ifndef RECORD_GAP_MICROS_WARNING_THRESHOLD
#define RECORD_GAP_MICROS_WARNING_THRESHOLD 20000
#endif

static constexpr unsigned int RECORD_GAP_TICKS = RECORD_GAP_MICROS / MICROS_PER_TICK;

// -------------------- Input polarity --------------------
#ifdef IR_INPUT_IS_ACTIVE_HIGH
#define INPUT_MARK 1
#else
#define INPUT_MARK 0
#endif

// -------------------- Sending options --------------------
#if defined(SEND_PWM_BY_TIMER) && defined(USE_NO_SEND_PWM)
#undef SEND_PWM_BY_TIMER
#warning "SEND_PWM_BY_TIMER and USE_NO_SEND_PWM both defined -> SEND_PWM_BY_TIMER disabled"
#endif

#ifndef PULSE_CORRECTION_NANOS
# if defined(F_CPU)
#  define PULSE_CORRECTION_NANOS (48000000000L / F_CPU)
# else
#  define PULSE_CORRECTION_NANOS 600
# endif
#endif

// -------------------- Core declarations --------------------
#include "IRremoteInt.h"

// -------------------- Implementation mode --------------------
// Use this in ONE file only:
//
// #define NEZAIR_IMPLEMENTATION
// #include <NezaIR.h>
//
// Do not define NEZAIR_IMPLEMENTATION in more than one translation unit.

#ifdef NEZAIR_IMPLEMENTATION

#include "private/IRTimer.hpp"
#include "IRFeedbackLED.hpp"
#include "IRReceive.hpp"
#include "IRSend.hpp"

IRrecv IrReceiver;
IRsend IrSender;

#endif // NEZAIR_IMPLEMENTATION

// -------------------- Legacy compatibility macros --------------------
#ifndef RAWBUF
#define RAWBUF RAW_BUFFER_LENGTH
#endif

#define REPEAT 0xFFFFFFFF
#define USECPERTICK MICROS_PER_TICK
#define MARK_EXCESS MARK_EXCESS_MICROS

#endif // NEZAIR_H
