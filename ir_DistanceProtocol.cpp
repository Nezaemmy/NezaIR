#include <Arduino.h>

// Accept durations up to DURATION_ARRAY_SIZE * MICROS_PER_TICK (default: 50 * 50 = 2500 µs)
#define DURATION_ARRAY_SIZE 50

// Switch the decoding bit order to match your target protocol.
// LSB_FIRST matches NEC and Kaseikyo/Panasonic; MSB_FIRST matches JVC, Denon.
#define DISTANCE_DO_MSB_DECODING PROTOCOL_IS_LSB_FIRST
//#define DISTANCE_DO_MSB_DECODING PROTOCOL_IS_MSB_FIRST

// INFO output is enabled by default but can be suppressed by defining NO_INFO before including.
#ifndef NO_INFO
#  ifndef INFO
#    define INFO
#  endif
#endif

//#define DEBUG  // Uncomment for verbose decode debug output from this file.

#include "IRremoteInt.h"  // evaluates DEBUG / INFO for DEBUG_PRINT / INFO_PRINT macros

/** \addtogroup Decoder Decoders and encoders for different protocols
 * @{
 */
// Protocol reference: https://www.mikrocontroller.net/articles/IRMP_-_english#Codings

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

#if defined(DEBUG)
/**
 * Dump a duration histogram array to Serial.
 * Only compiled in when DEBUG is active.
 */
static void printDurations(uint8_t aArray[], uint8_t aMaxIndex) {
    for (uint_fast8_t i = 0; i <= aMaxIndex; i++) {
        if (i % 10 == 0) {
            if (i == 0) {
                Serial.print(' ');  // indent the leading "0"
            } else {
                Serial.println();
            }
            Serial.print(i);
            Serial.print(F(":"));
        }
        Serial.print(F(" | "));
        Serial.print(aArray[i]);
    }
    Serial.println();
}
#endif  // DEBUG

/**
 * Aggregate contiguous non-zero runs in aArray[] into single weighted-average
 * bins and record the two resulting bin indices as aShortIndex / aLongIndex.
 *
 * @return true  – at most 2 distinct duration clusters found (pulse-width or
 *                 pulse-distance protocol likely)
 * @return false – 3 or more clusters found (e.g. RC5); caller should give up
 */
static bool aggregateArrayCounts(uint8_t aArray[], uint8_t aMaxIndex,
                                 uint8_t *aShortIndex, uint8_t *aLongIndex) {
    uint8_t  tSum         = 0;
    uint16_t tWeightedSum = 0;

    for (uint_fast8_t i = 0; i <= aMaxIndex; i++) {
        uint8_t tCurrentDurations = aArray[i];
        if (tCurrentDurations != 0) {
            tSum         += tCurrentDurations;
            tWeightedSum += (uint16_t)(tCurrentDurations * i);
            aArray[i]     = 0;
        }

        // End of a run: either a zero gap or the last element with pending sum
        if ((tCurrentDurations == 0 || i == aMaxIndex) && tSum != 0) {
            uint8_t tAggregateIndex = (uint8_t)((tWeightedSum + (tSum / 2)) / tSum);  // rounded average
            aArray[tAggregateIndex] = tSum;  // write back aggregate (removing this line costs +2 bytes — compiler quirk)

            if (*aShortIndex == 0) {
                *aShortIndex = tAggregateIndex;
            } else if (*aLongIndex == 0) {
                *aLongIndex = tAggregateIndex;
            } else {
                // Three or more bins → not a simple pulse-width / distance protocol
                return false;
            }

            tSum         = 0;
            tWeightedSum = 0;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// IRrecv::decodeDistance
// ---------------------------------------------------------------------------

/*
 * Try to decode a pulse-width or pulse-distance protocol:
 *  1. Analyse all space and mark lengths and fill histogram arrays.
 *  2. Aggregate histogram bins — expect at most 2 distinct durations each.
 *  3. Decide whether we have pulse-width or pulse-distance encoding.
 *  4. Decode up to 32 bits per pass (repeated for payloads > 32 bits).
 *
 * Result: raw data only — no address / command split.
 */
bool IRrecv::decodeDistance() {
    uint8_t tDurationArray[DURATION_ARRAY_SIZE];

    // Retrieve the active tick resolution (may differ per receiver instance)
    const int iMICROS_PER_TICK = getMICROS_PER_TICK();

    // Require at least 8 data bits: 2*8 transitions + 2 header + 2 stop = 20
    if (decodedIRData.rawDataPtr->rawlen < (2 * 8) + 4) {
        DEBUG_PRINT(F("PULSE_DISTANCE: "));
        DEBUG_PRINT(F("Data length="));
        DEBUG_PRINT(decodedIRData.rawDataPtr->rawlen);
        DEBUG_PRINTLN(F(" is less than 20"));
        return false;
    }

    // ------------------------------------------------------------------
    // 1a. Histogram space durations (even indices; skip header & stop)
    // ------------------------------------------------------------------
    memset(tDurationArray, 0, sizeof(tDurationArray));
    uint8_t tMaxDurationIndex = 0;

    for (uint_fast8_t i = 4; i < (uint_fast8_t)(decodedIRData.rawDataPtr->rawlen - 2); i += 2) {
        uint8_t tDurationTicks = decodedIRData.rawDataPtr->rawbuf[i];
        if (tDurationTicks < DURATION_ARRAY_SIZE) {  // use named constant, not sizeof
            tDurationArray[tDurationTicks]++;
            if (tMaxDurationIndex < tDurationTicks) {
                tMaxDurationIndex = tDurationTicks;
            }
        }
    }

    uint8_t tSpaceTicksShort = 0;
    uint8_t tSpaceTicksLong  = 0;
    if (!aggregateArrayCounts(tDurationArray, tMaxDurationIndex, &tSpaceTicksShort, &tSpaceTicksLong)) {
        DEBUG_PRINT(F("PULSE_DISTANCE: "));
        DEBUG_PRINTLN(F("Space aggregation failed, more than 2 distinct duration values found"));
        return false;
    }

#if defined(DEBUG)
    Serial.println(F("Space:"));
    printDurations(tDurationArray, tMaxDurationIndex);
#endif

    // ------------------------------------------------------------------
    // 1b. Histogram mark durations (odd indices; skip header & stop)
    // ------------------------------------------------------------------
    memset(tDurationArray, 0, sizeof(tDurationArray));
    tMaxDurationIndex = 0;

    for (uint_fast8_t i = 3; i < (uint_fast8_t)(decodedIRData.rawDataPtr->rawlen - 2); i += 2) {
        uint8_t tDurationTicks = decodedIRData.rawDataPtr->rawbuf[i];
        if (tDurationTicks < DURATION_ARRAY_SIZE) {  // use named constant, not sizeof
            tDurationArray[tDurationTicks]++;
            if (tMaxDurationIndex < tDurationTicks) {
                tMaxDurationIndex = tDurationTicks;
            }
        }
    }

    uint8_t tMarkTicksShort = 0;
    uint8_t tMarkTicksLong  = 0;
    if (!aggregateArrayCounts(tDurationArray, tMaxDurationIndex, &tMarkTicksShort, &tMarkTicksLong)) {
        DEBUG_PRINT(F("PULSE_DISTANCE: "));
        DEBUG_PRINTLN(F("Mark aggregation failed, more than 2 distinct duration values found"));
        return false;  // BUG FIX: was missing — function previously continued with garbage tick values
    }

#if defined(DEBUG)
    Serial.println(F("Mark:"));
    printDurations(tDurationArray, tMaxDurationIndex);
#endif

    // ------------------------------------------------------------------
    // 2. Compute bit count and multi-word decode parameters
    // ------------------------------------------------------------------
    // rawlen/2 gives total mark+space pairs; subtract 2 for header and stop.
    uint16_t tNumberOfBits = (uint16_t)(decodedIRData.rawDataPtr->rawlen / 2) - 2;
    uint8_t  tStartIndex   = 3;

    decodedIRData.numberOfBits = tNumberOfBits;

    // Number of additional 32-bit words needed beyond the first
    // Formula: ceil(tNumberOfBits / 32) - 1  ==  (tNumberOfBits - 1) / 32
    uint8_t tNumberOfAdditionalLong = (uint8_t)((tNumberOfBits - 1) / 32);

    // ------------------------------------------------------------------
    // 3 & 4. Decode — pulse-width or pulse-distance
    // ------------------------------------------------------------------
    if (tSpaceTicksLong == 0) {
        // ---- Pulse-WIDTH protocol (mark length encodes the bit) ----
        if (tMarkTicksLong == 0) {
            DEBUG_PRINT(F("PULSE_DISTANCE: "));
            DEBUG_PRINTLN(F("Only 1 distinct duration value for each space and mark found"));
            return false;
        }

        for (uint8_t i = 0; i <= tNumberOfAdditionalLong; ++i) {
            uint8_t tBitsThisPass = (tNumberOfBits > 32) ? 32 : (uint8_t)tNumberOfBits;

            if (!decodePulseWidthData(tBitsThisPass, tStartIndex,
                                      tMarkTicksLong  * iMICROS_PER_TICK,
                                      tMarkTicksShort * iMICROS_PER_TICK,
                                      tSpaceTicksShort * iMICROS_PER_TICK,
                                      DISTANCE_DO_MSB_DECODING)) {
                DEBUG_PRINT(F("PULSE_WIDTH: "));
                DEBUG_PRINTLN(F("Decode failed"));
                return false;
            }

            if (i == 0) {
                // Print header timing once at the start of the first word
                INFO_PRINTLN();
                INFO_PRINT(F("PULSE_WIDTH:"));
                INFO_PRINT(F(" HeaderMarkMicros="));
                INFO_PRINT(decodedIRData.rawDataPtr->rawbuf[1] * iMICROS_PER_TICK);
                INFO_PRINT(F(" HeaderSpaceMicros="));
                INFO_PRINT(decodedIRData.rawDataPtr->rawbuf[2] * iMICROS_PER_TICK);
                INFO_PRINT(F(" OneMarkMicros="));
                INFO_PRINT(tMarkTicksLong  * iMICROS_PER_TICK);
                INFO_PRINT(F(" ZeroMarkMicros="));
                INFO_PRINT(tMarkTicksShort * iMICROS_PER_TICK);
                INFO_PRINT(F(" SpaceMicros="));
                INFO_PRINTLN(tSpaceTicksShort * iMICROS_PER_TICK);
            }

            if (tNumberOfAdditionalLong > 0) {
                INFO_PRINT(F(" 0x"));
                INFO_PRINT(decodedIRData.decodedRawData, HEX);
                tStartIndex    += 64;
                tNumberOfBits  -= 32;
            }
        }

        decodedIRData.extra    = (uint16_t)((tMarkTicksShort << 8) | tMarkTicksLong);
        decodedIRData.protocol = PULSE_WIDTH;

    } else {
        // ---- Pulse-DISTANCE protocol (space length encodes the bit) ----
        for (uint8_t i = 0; i <= tNumberOfAdditionalLong; ++i) {
            uint8_t tBitsThisPass = (tNumberOfBits > 32) ? 32 : (uint8_t)tNumberOfBits;

            if (!decodePulseDistanceData(tBitsThisPass, tStartIndex,
                                         tMarkTicksShort  * iMICROS_PER_TICK,
                                         tSpaceTicksLong  * iMICROS_PER_TICK,
                                         tSpaceTicksShort * iMICROS_PER_TICK,
                                         DISTANCE_DO_MSB_DECODING)) {
                DEBUG_PRINT(F("PULSE_DISTANCE: "));
                DEBUG_PRINTLN(F("Decode failed"));
                return false;
            }

            if (i == 0) {
                // Print header timing once at the start of the first word
                INFO_PRINTLN();
                INFO_PRINT(F("PULSE_DISTANCE:"));
                INFO_PRINT(F(" HeaderMarkMicros="));
                INFO_PRINT(decodedIRData.rawDataPtr->rawbuf[1] * iMICROS_PER_TICK);
                INFO_PRINT(F(" HeaderSpaceMicros="));
                INFO_PRINT(decodedIRData.rawDataPtr->rawbuf[2] * iMICROS_PER_TICK);
                INFO_PRINT(F(" MarkMicros="));
                INFO_PRINT(tMarkTicksShort  * iMICROS_PER_TICK);
                INFO_PRINT(F(" OneSpaceMicros="));
                INFO_PRINT(tSpaceTicksLong  * iMICROS_PER_TICK);
                INFO_PRINT(F(" ZeroSpaceMicros="));
                INFO_PRINTLN(tSpaceTicksShort * iMICROS_PER_TICK);
            }

            if (tNumberOfAdditionalLong > 0) {
                INFO_PRINT(F(" 0x"));
                INFO_PRINT(decodedIRData.decodedRawData, HEX);
                tStartIndex   += 64;
                tNumberOfBits -= 32;
            }
        }

        decodedIRData.extra    = (uint16_t)((tSpaceTicksShort << 8) | tSpaceTicksLong);
        decodedIRData.protocol = PULSE_DISTANCE;
    }

    if (DISTANCE_DO_MSB_DECODING) {
        decodedIRData.flags = IRDATA_FLAGS_IS_MSB_FIRST;
    }

    return true;
}

/** @} */
