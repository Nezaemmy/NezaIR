#pragma once
#ifndef NezaIR_h
#define NezaIR_h

// -------------------- Version (prefer unique names) --------------------
#define VERSION_NEZAIR "2.1.2"
#define VERSION_NEZAIR_MAJOR 2
#define VERSION_NEZAIR_MINOR 1

// -------------------- Multi-IR defaults (override in sketch before include) --------------------
#ifndef IR_MAXCOUNTIR
#define IR_MAXCOUNTIR 8
#endif

#ifndef MICROS_PER_TICK
#define MICROS_PER_TICK 150
#endif

#ifndef RAW_BUFFER_LENGTH
#define RAW_BUFFER_LENGTH 100    // reduce if you need more SRAM
#endif

// Optional: exclude exotic protocols by default to save flash
// Uncomment if you don't need BOSEWAVE, MAGIQUEST, WHYNTER, LEGO_PF
// #define EXCLUDE_EXOTIC_PROTOCOLS

// -------------------- Protocol selection defaults --------------------
// If user did not define any DECODE_* macros, enable a default set.
#if (!(defined(DECODE_DENON)       || defined(DECODE_JVC)        || defined(DECODE_KASEIKYO) || defined(DECODE_PANASONIC) || \
       defined(DECODE_LG)          || defined(DECODE_NEC)        || defined(DECODE_SAMSUNG)  || defined(DECODE_SONY)      || \
       defined(DECODE_RC5)         || defined(DECODE_RC6)        || defined(DECODE_HASH)     || defined(DECODE_DISTANCE)  || \
       defined(DECODE_BOSEWAVE)    || defined(DECODE_LEGO_PF)    || defined(DECODE_MAGIQUEST)|| defined(DECODE_WHYNTER)))

#define DECODE_NEC
#define DECODE_SONY
#define DECODE_SAMSUNG
#define DECODE_RC5
#define DECODE_RC6

// Universal decoders (good for "any remote")
#ifndef EXCLUDE_UNIVERSAL_PROTOCOLS
#define DECODE_DISTANCE
#define DECODE_HASH
#endif

// Optional extra protocols (uncomment if needed)
// #define DECODE_LG
// #define DECODE_JVC
// #define DECODE_DENON
// #define DECODE_KASEIKYO
// #define DECODE_PANASONIC

#ifndef EXCLUDE_EXOTIC_PROTOCOLS
// #define DECODE_BOSEWAVE
// #define DECODE_LEGO_PF
// #define DECODE_MAGIQUEST
// #define DECODE_WHYNTER
#endif

#endif // default decode set

// -------------------- Receiver tuning (user may override) --------------------
#ifndef MARK_EXCESS_MICROS
#define MARK_EXCESS_MICROS 20
#endif

// Minimum number of ticks a mark or space must last to be considered a real
// IR signal rather than noise. Pulses shorter than this are silently discarded
// by the ISR. This prevents a floating or reconnected pin from flooding the
// CPU with spurious interrupts and blocking the ESP32.
// Default: 2 ticks = 2 * MICROS_PER_TICK (e.g. 2 * 150µs = 300µs).
// Real IR protocol marks/spaces are always >= 300µs, so this is safe.
// Set to 0 to disable filtering (not recommended on ESP32).
#ifndef MIN_SIGNAL_TICKS
#define MIN_SIGNAL_TICKS 2
#endif

#ifndef RECORD_GAP_MICROS
#define RECORD_GAP_MICROS 5000
#endif

#ifndef RECORD_GAP_MICROS_WARNING_THRESHOLD
#define RECORD_GAP_MICROS_WARNING_THRESHOLD 20000
#endif

// Use constexpr for type-safe compile-time constant (requires C++11)
static constexpr unsigned int RECORD_GAP_TICKS = RECORD_GAP_MICROS / MICROS_PER_TICK;

// Input polarity
// #define IR_INPUT_IS_ACTIVE_HIGH
#ifdef IR_INPUT_IS_ACTIVE_HIGH
#define INPUT_MARK 1
#else
#define INPUT_MARK 0
#endif

// -------------------- Sending options --------------------
// #define SEND_PWM_BY_TIMER
// #define USE_NO_SEND_PWM
#if defined(SEND_PWM_BY_TIMER) && defined(USE_NO_SEND_PWM)
#undef SEND_PWM_BY_TIMER
#warning "SEND_PWM_BY_TIMER and USE_NO_SEND_PWM both defined -> SEND_PWM_BY_TIMER disabled"
#endif

#ifndef PULSE_CORRECTION_NANOS
#  if defined(F_CPU)
#    define PULSE_CORRECTION_NANOS (48000000000L / F_CPU)
#  else
#    define PULSE_CORRECTION_NANOS 600
#  endif
#endif

// -------------------- Library includes --------------------
#include "IRremoteInt.h"
#include "private/IRTimer.hpp"
#include "IRFeedbackLED.hpp"
#include "IRReceive.hpp"
#include "IRSend.hpp"

// -------------------- Legacy compatibility macros (deprecated aliases) --------------------
// These exist for backwards compatibility only. Prefer the modern names above.
#ifndef RAWBUF
#define RAWBUF RAW_BUFFER_LENGTH
#endif

#define REPEAT          0xFFFFFFFF
#define USECPERTICK     MICROS_PER_TICK
#define MARK_EXCESS     MARK_EXCESS_MICROS

#endif // NezaIR_h
