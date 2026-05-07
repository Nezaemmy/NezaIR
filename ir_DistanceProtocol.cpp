#include <Arduino.h>
#include <string.h>
#include "IRremoteInt.h"

#ifndef DURATION_ARRAY_SIZE
#define DURATION_ARRAY_SIZE 50
#endif

#ifndef DISTANCE_DECODE_MSB_FIRST
#define DISTANCE_DECODE_MSB_FIRST false
#endif

/** \addtogroup Decoder Decoders and encoders for different protocols
 * @{
 */

#if defined(DEBUG)

static void printDurations(uint8_t aArray[], uint8_t aMaxIndex) {
    for (uint_fast8_t i = 0; i <= aMaxIndex; i++) {
        if (i % 10 == 0) {
            if (i == 0) {
                Serial.print(' ');
            } else {
                Serial.println();
            }

            Serial.print(i);
            Serial.print(F(":"));
        }

        Serial.print(F(" "));
        Serial.print(aArray[i]);
    }

    Serial.println();
}

#endif // DEBUG

static bool aggregateArrayCounts(uint8_t aArray[],
                                 uint8_t aMaxIndex,
                                 uint8_t *aShortIndex,
                                 uint8_t *aLongIndex) {
    uint8_t clusterCount = 0;
    uint8_t clusterIndex[2] = {0, 0};

    uint8_t i = 0;

    while (i <= aMaxIndex) {
        while (i <= aMaxIndex && aArray[i] == 0) {
            i++;
        }

        if (i > aMaxIndex) break;

        uint16_t weightedSum = 0;
        uint16_t countSum    = 0;

        while (i <= aMaxIndex && aArray[i] != 0) {
            weightedSum += (uint16_t)aArray[i] * i;
            countSum    += aArray[i];
            i++;
        }

        if (countSum == 0) continue;

        if (clusterCount >= 2) {
            return false;
        }

        clusterIndex[clusterCount++] = (uint8_t)((weightedSum + (countSum / 2)) / countSum);
    }

    if (clusterCount == 0) {
        return false;
    }

    if (clusterCount == 1) {
        *aShortIndex = clusterIndex[0];
        *aLongIndex  = clusterIndex[0];
        return true;
    }

    if (clusterIndex[0] <= clusterIndex[1]) {
        *aShortIndex = clusterIndex[0];
        *aLongIndex  = clusterIndex[1];
    } else {
        *aShortIndex = clusterIndex[1];
        *aLongIndex  = clusterIndex[0];
    }

    return true;
}

static bool fillDurationHistogram(IRData *aIRData,
                                  uint8_t aStartIndex,
                                  uint8_t aStep,
                                  uint8_t aDurationArray[],
                                  uint8_t *aMaxDurationIndex) {
    memset(aDurationArray, 0, DURATION_ARRAY_SIZE);

    *aMaxDurationIndex = 0;

    if (aIRData == nullptr || aIRData->rawDataPtr == nullptr) return false;

    uint16_t rawlen = aIRData->rawDataPtr->rawlen;

    if (rawlen <= aStartIndex) return false;

    for (uint16_t i = aStartIndex; i < rawlen; i += aStep) {
        uint16_t durationTicks = aIRData->rawDataPtr->rawbuf[i];

        if (durationTicks >= DURATION_ARRAY_SIZE) {
            return false;
        }

        aDurationArray[durationTicks]++;

        if (*aMaxDurationIndex < durationTicks) {
            *aMaxDurationIndex = durationTicks;
        }
    }

    return true;
}

bool IRrecv::decodeDistance() {
#if !defined(DECODE_DISTANCE)
    return false;
#else
    if (decodedIRData.rawDataPtr == nullptr) return false;

    const uint16_t rawlen = decodedIRData.rawDataPtr->rawlen;

    /*
     * rawbuf layout:
     * [0] gap
     * [1] header mark
     * [2] header space
     * [3] first bit mark
     * [4] first bit space
     * ...
     */
    if (rawlen < 20) {
        DEBUG_PRINT(F("DISTANCE: "));
        DEBUG_PRINT(F("rawlen="));
        DEBUG_PRINT(rawlen);
        DEBUG_PRINTLN(F(" is too short"));
        return false;
    }

    uint8_t markDurationArray[DURATION_ARRAY_SIZE];
    uint8_t spaceDurationArray[DURATION_ARRAY_SIZE];

    uint8_t maxMarkIndex  = 0;
    uint8_t maxSpaceIndex = 0;

    /*
     * Data marks start at index 3.
     * Data spaces start at index 4.
     */
    if (!fillDurationHistogram(&decodedIRData, 3, 2, markDurationArray, &maxMarkIndex)) {
        DEBUG_PRINTLN(F("DISTANCE: mark histogram failed"));
        return false;
    }

    if (!fillDurationHistogram(&decodedIRData, 4, 2, spaceDurationArray, &maxSpaceIndex)) {
        DEBUG_PRINTLN(F("DISTANCE: space histogram failed"));
        return false;
    }

#if defined(DEBUG)
    Serial.println(F("DISTANCE mark histogram:"));
    printDurations(markDurationArray, maxMarkIndex);

    Serial.println(F("DISTANCE space histogram:"));
    printDurations(spaceDurationArray, maxSpaceIndex);
#endif

    uint8_t markTicksShort  = 0;
    uint8_t markTicksLong   = 0;
    uint8_t spaceTicksShort = 0;
    uint8_t spaceTicksLong  = 0;

    if (!aggregateArrayCounts(markDurationArray, maxMarkIndex,
                              &markTicksShort, &markTicksLong)) {
        DEBUG_PRINTLN(F("DISTANCE: mark aggregation failed"));
        return false;
    }

    if (!aggregateArrayCounts(spaceDurationArray, maxSpaceIndex,
                              &spaceTicksShort, &spaceTicksLong)) {
        DEBUG_PRINTLN(F("DISTANCE: space aggregation failed"));
        return false;
    }

    const bool marksAreDifferent  = (markTicksShort != markTicksLong);
    const bool spacesAreDifferent = (spaceTicksShort != spaceTicksLong);

    /*
     * Number of bits:
     * rawlen includes gap/header.
     * Data starts at rawbuf[3] as mark/space pairs.
     */
    uint8_t numberOfBits = (uint8_t)((rawlen - 3) / 2);

    if (numberOfBits == 0 || numberOfBits > 32) {
        DEBUG_PRINTLN(F("DISTANCE: invalid bit count"));
        return false;
    }

    const uint16_t tickMicros = (uint16_t)getMICROS_PER_TICK();

    bool decoded = false;

    if (!marksAreDifferent && spacesAreDifferent) {
        /*
         * Pulse-distance protocol:
         * bit value is encoded in the space duration.
         */
        decoded = decodePulseDistanceData(
            numberOfBits,
            3,
            (uint16_t)markTicksShort * tickMicros,
            (uint16_t)spaceTicksLong * tickMicros,
            (uint16_t)spaceTicksShort * tickMicros,
            DISTANCE_DECODE_MSB_FIRST
        );
    } else if (marksAreDifferent && !spacesAreDifferent) {
        /*
         * Pulse-width protocol:
         * bit value is encoded in the mark duration.
         */
        decoded = decodePulseWidthData(
            numberOfBits,
            3,
            (uint16_t)markTicksLong * tickMicros,
            (uint16_t)markTicksShort * tickMicros,
            (uint16_t)spaceTicksShort * tickMicros,
            DISTANCE_DECODE_MSB_FIRST
        );
    } else {
        /*
         * Both mark and space have two clusters, or both have one cluster.
         * This is ambiguous for this generic decoder.
         */
        DEBUG_PRINTLN(F("DISTANCE: ambiguous mark/space clusters"));
        return false;
    }

    if (!decoded) {
        DEBUG_PRINTLN(F("DISTANCE: primitive decoder failed"));
        return false;
    }

    decodedIRData.protocol     = DISTANCE;
    decodedIRData.address      = 0;
    decodedIRData.command      = 0;
    decodedIRData.extra        = rawlen;
    decodedIRData.numberOfBits = numberOfBits;

    if (DISTANCE_DECODE_MSB_FIRST) {
        decodedIRData.flags = IRDATA_FLAGS_IS_MSB_FIRST;
    } else {
        decodedIRData.flags = IRDATA_FLAGS_IS_LSB_FIRST;
    }

    return true;
#endif
}

/** @} */
