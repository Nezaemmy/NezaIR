
#include <Arduino.h>

//#define DEBUG // Activate this for lots of lovely debug output from this decoder.
#include "IRremoteInt.h" // evaluates the DEBUG for DEBUG_PRINT


#define DENON_ADDRESS_BITS      5
#define DENON_COMMAND_BITS      8
#define DENON_FRAME_BITS        2 // 00/10 for 1. frame Denon/Sharp, inverted for autorepeat frame

#define DENON_BITS              (DENON_ADDRESS_BITS + DENON_COMMAND_BITS + DENON_FRAME_BITS) // 15 - The number of bits in the command
#define DENON_UNIT              260

#define DENON_BIT_MARK          DENON_UNIT  // The length of a Bit:Mark
#define DENON_ONE_SPACE         (7 * DENON_UNIT) // 1820 // The length of a Bit:Space for 1's
#define DENON_ZERO_SPACE        (3 * DENON_UNIT) // 780 // The length of a Bit:Space for 0's

#define DENON_AUTO_REPEAT_SPACE 45000 // Every frame is auto repeated with a space period of 45 ms and the command inverted.
#define DENON_REPEAT_PERIOD     110000 // Commands are repeated every 110 ms (measured from start to start) for as long as the key on the remote control is held down.

// for old decoder
#define DENON_HEADER_MARK       DENON_UNIT // The length of the Header:Mark
#define DENON_HEADER_SPACE      (3 * DENON_UNIT) // 780 // The length of the Header:Space

//+=============================================================================
void IRsend::sendSharp(uint8_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats) {
    sendDenon(aAddress, aCommand, aNumberOfRepeats, true);
}

/*
 * Only for backwards compatibility
 */
void IRsend::sendDenonRaw(uint16_t aRawData, uint_fast8_t aNumberOfRepeats) {
    sendDenon(aRawData >> (DENON_COMMAND_BITS + DENON_FRAME_BITS), aRawData & 0xFF, aNumberOfRepeats);
}

//+=============================================================================
void IRsend::sendDenon(uint8_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats, bool aSendSharp) {
    // Set IR carrier frequency
    enableIROut(DENON_KHZ); // 38 kHz

    // Shift command and add frame marker
    uint16_t tCommand = aCommand << DENON_FRAME_BITS; // the lowest bits are 00 for Denon and 10 for Sharp
    if (aSendSharp) {
        tCommand |= 0x02;
    }
    uint16_t tData = tCommand | ((uint16_t) aAddress << (DENON_COMMAND_BITS + DENON_FRAME_BITS));
    uint16_t tInvertedData = ((~tCommand) & 0x3FF) | (uint16_t) aAddress << (DENON_COMMAND_BITS + DENON_FRAME_BITS);

    uint_fast8_t tNumberOfCommands = aNumberOfRepeats + 1;
    while (tNumberOfCommands > 0) {

        // Data
        sendPulseDistanceWidthData(DENON_BIT_MARK, DENON_ONE_SPACE, DENON_BIT_MARK, DENON_ZERO_SPACE, tData, DENON_BITS, PROTOCOL_IS_MSB_FIRST,
        SEND_STOP_BIT);

        // Inverted autorepeat frame
        delay(DENON_AUTO_REPEAT_SPACE / 1000);
        sendPulseDistanceWidthData(DENON_BIT_MARK, DENON_ONE_SPACE, DENON_BIT_MARK, DENON_ZERO_SPACE, tInvertedData, DENON_BITS,
        PROTOCOL_IS_MSB_FIRST, SEND_STOP_BIT);

        tNumberOfCommands--;
        // skip last delay!
        if (tNumberOfCommands > 0) {
            // send repeated command with a fixed space gap
            delay( DENON_AUTO_REPEAT_SPACE / 1000);
        }
    }
}

//+=============================================================================
bool IRrecv::decodeSharp() {
    return decodeDenon();
}

//+=============================================================================
bool IRrecv::decodeDenon() {

    // we have no start bit, so check for the exact amount of data bits
    // Check we have the right amount of data (32). The + 2 is for initial gap + stop bit mark
    if (decodedIRData.rawDataPtr->rawlen != (2 * DENON_BITS) + 2) {
        DEBUG_PRINT(F("Denon: "));
        DEBUG_PRINT("Data length=");
        DEBUG_PRINT(decodedIRData.rawDataPtr->rawlen);
        DEBUG_PRINTLN(" is not 32");
        return false;
    }

    // Read the bits in
    if (!decodePulseDistanceData(DENON_BITS, 1, DENON_BIT_MARK, DENON_ONE_SPACE, DENON_ZERO_SPACE, PROTOCOL_IS_MSB_FIRST)) {
        DEBUG_PRINT("Denon: ");
        DEBUG_PRINTLN("Decode failed");
        return false;
    }

    // Check for stop mark
    if (!matchMark(decodedIRData.rawDataPtr->rawbuf[(2 * DENON_BITS) + 1], DENON_HEADER_MARK)) {
        DEBUG_PRINT("Denon: ");
        DEBUG_PRINTLN(F("Stop bit mark length is wrong"));
        return false;
    }

    // Success
    decodedIRData.flags = IRDATA_FLAGS_IS_MSB_FIRST;
    uint8_t tFrameBits = decodedIRData.decodedRawData & 0x03;
    decodedIRData.command = decodedIRData.decodedRawData >> DENON_FRAME_BITS;
    decodedIRData.address = decodedIRData.command >> DENON_COMMAND_BITS;
    uint8_t tCommand = decodedIRData.command & 0xFF;
    decodedIRData.command = tCommand;

    // check for autorepeated inverted command
    int iMICROS_PER_TICK = getMICROS_PER_TICK(); //Modified to get the correct iMICROS_PER_TICK
    if (decodedIRData.rawDataPtr->rawbuf[0] < ((DENON_AUTO_REPEAT_SPACE + (DENON_AUTO_REPEAT_SPACE / 4)) / iMICROS_PER_TICK)) {
        repeatCount++;
        if (tFrameBits == 0x3 || tFrameBits == 0x1) {
            // We are in the auto repeated frame with the inverted command
            decodedIRData.flags = IRDATA_FLAGS_IS_AUTO_REPEAT | IRDATA_FLAGS_IS_MSB_FIRST;
            // Check parity of consecutive received commands. There is no parity in one data set.
            uint8_t tLastCommand = lastDecodedCommand;
            if (tLastCommand != (uint8_t) (~tCommand)) {
                decodedIRData.flags |= IRDATA_FLAGS_PARITY_FAILED;
            }
            // always take non inverted command
            decodedIRData.command = tLastCommand;
        }
        if (repeatCount > 1) {
            decodedIRData.flags |= IRDATA_FLAGS_IS_REPEAT;
        }
    } else {
        repeatCount = 0;
    }

    decodedIRData.numberOfBits = DENON_BITS;
    if (tFrameBits == 1 || tFrameBits == 2) {
        decodedIRData.protocol = SHARP;
    } else {
        decodedIRData.protocol = DENON;
    }
    return true;
}

#if !defined(NO_LEGACY_COMPATIBILITY)
bool IRrecv::decodeDenonOld(decode_results *aResults) {

    // Check we have the right amount of data
    if (decodedIRData.rawDataPtr->rawlen != 1 + 2 + (2 * DENON_BITS) + 1) {
        return false;
    }

    // Check initial Mark+Space match
    if (!matchMark(aResults->rawbuf[1], DENON_HEADER_MARK)) {
        return false;
    }

    if (!matchSpace(aResults->rawbuf[2], DENON_HEADER_SPACE)) {
        return false;
    }

    // Read the bits in
    if (!decodePulseDistanceData(DENON_BITS, 3, DENON_BIT_MARK, DENON_ONE_SPACE, DENON_ZERO_SPACE, PROTOCOL_IS_MSB_FIRST)) {
        return false;
    }

    // Success
    aResults->value = decodedIRData.decodedRawData;
    aResults->bits = DENON_BITS;
    aResults->decode_type = DENON;
    decodedIRData.protocol = DENON;
    return true;
}
#endif

void IRsend::sendDenon(unsigned long data, int nbits) {
    // Set IR carrier frequency
    enableIROut(DENON_KHZ);
    Serial.println(
            "The function sendDenon(data, nbits) is deprecated and may not work as expected! Use sendDenonRaw(data, NumberOfRepeats) or better sendDenon(Address, Command, NumberOfRepeats).");

    // Header
    mark(DENON_HEADER_MARK);
    space(DENON_HEADER_SPACE);

    // Data
    sendPulseDistanceWidthData(DENON_BIT_MARK, DENON_ONE_SPACE, DENON_BIT_MARK, DENON_ZERO_SPACE, data, nbits, PROTOCOL_IS_MSB_FIRST,
    SEND_STOP_BIT);

}

void IRsend::sendSharp(unsigned int aAddress, unsigned int aCommand) {
    sendDenon(aAddress, aCommand, true, 0);
}

/** @}*/
