
#include <Arduino.h>

//#define DEBUG // Activate this for lots of lovely debug output from this decoder.
#include "IRremoteInt.h" // evaluates the DEBUG for DEBUG_PRINT



// LSB first, 1 start bit + 16 bit address + 8 bit command + 1 stop bit.
#define SHUZU_ADDRESS_BITS      16 // 16 bit address
#define SHUZU_COMMAND_BITS      8 // Command

#define SHUZU_BITS              (SHUZU_ADDRESS_BITS + SHUZU_COMMAND_BITS) // The number of bits in the protocol
#define SHUZU_UNIT              560

#define SHUZU_HEADER_MARK       (16 * SHUZU_UNIT) // The length of the Header:Mark
#define SHUZU_HEADER_SPACE      (8 * SHUZU_UNIT)  // The length of the Header:Space

#define SHUZU_BIT_MARK          SHUZU_UNIT        // The length of a Bit:Mark
#define SHUZU_ONE_SPACE         (3 * SHUZU_UNIT)  // The length of a Bit:Space for 1's
#define SHUZU_ZERO_SPACE        SHUZU_UNIT        // The length of a Bit:Space for 0's

#define SHUZU_REPEAT_HEADER_SPACE (4 * SHUZU_UNIT)  // 2250

#define SHUZU_REPEAT_SPACE      45000

#define SHUZU_OTHER             1234  // Other things you may need to define

//+=============================================================================
//
void IRsend::sendShuzu(uint16_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats) {
    // Set IR carrier frequency
    enableIROut(38);

    uint_fast8_t tNumberOfCommands = aNumberOfRepeats + 1;
    while (tNumberOfCommands > 0) {

        // Header
        mark(SHUZU_HEADER_MARK);
        space(SHUZU_HEADER_SPACE);

        // Address (device and subdevice)
        sendPulseDistanceWidthData(SHUZU_BIT_MARK, SHUZU_ONE_SPACE, SHUZU_BIT_MARK, SHUZU_ZERO_SPACE, aAddress,
        SHUZU_ADDRESS_BITS, PROTOCOL_IS_LSB_FIRST); // false -> LSB first

        // Command + stop bit
        sendPulseDistanceWidthData(SHUZU_BIT_MARK, SHUZU_ONE_SPACE, SHUZU_BIT_MARK, SHUZU_ZERO_SPACE, aCommand,
        SHUZU_COMMAND_BITS, PROTOCOL_IS_LSB_FIRST, SEND_STOP_BIT); // false, true -> LSB first, stop bit

        tNumberOfCommands--;
        // skip last delay!
        if (tNumberOfCommands > 0) {
            // send repeated command in a fixed raster
            delay(SHUZU_REPEAT_SPACE / 1000);
        }
    }
}

//+=============================================================================
//
/*
 * First check for right data length
 * Next check start bit
 * Next try the decode
 * Last check stop bit
 */
bool IRrecv::decodeShuzu() {

    // Check we have the right amount of data (28). The +4 is for initial gap, start bit mark and space + stop bit mark
    if (decodedIRData.rawDataPtr->rawlen != (2 * SHUZU_BITS) + 4) {
        // no debug output, since this check is mainly to determine the received protocol
        return false;
    }

    // Check header "space"
    if (!matchMark(decodedIRData.rawDataPtr->rawbuf[1], SHUZU_HEADER_MARK) || !matchSpace(decodedIRData.rawDataPtr->rawbuf[2], SHUZU_HEADER_SPACE)) {
        DEBUG_PRINT("Shuzu: ");
        DEBUG_PRINTLN("Header mark or space length is wrong");
        return false;
    }

    // false -> LSB first
    if (!decodePulseDistanceData(SHUZU_BITS, 3, SHUZU_BIT_MARK, SHUZU_ONE_SPACE, SHUZU_ZERO_SPACE, PROTOCOL_IS_LSB_FIRST)) {
        DEBUG_PRINT(F("Shuzu: "));
        DEBUG_PRINTLN(F("Decode failed"));
        return false;
    }

    // Stop bit
    if (!matchMark(decodedIRData.rawDataPtr->rawbuf[3 + (2 * SHUZU_BITS)], SHUZU_BIT_MARK)) {
        DEBUG_PRINT(F("Shuzu: "));
        DEBUG_PRINTLN(F("Stop bit mark length is wrong"));
        return false;
    }

    // Success
//    decodedIRData.flags = IRDATA_FLAGS_IS_LSB_FIRST; // Not required, since this is the start value
    uint8_t tCommand = decodedIRData.decodedRawData >> SHUZU_ADDRESS_BITS;  // upper 8 bits of LSB first value
    uint8_t tAddress = decodedIRData.decodedRawData & 0xFFFF;    // lowest 16 bit of LSB first value

    /*
     *  Check for repeat
     */
    int iMICROS_PER_TICK = getMICROS_PER_TICK(); //Modified to get the correct iMICROS_PER_TICK
    if (decodedIRData.rawDataPtr->rawbuf[0] < ((SHUZU_REPEAT_SPACE + (SHUZU_REPEAT_SPACE / 4)) / iMICROS_PER_TICK)) {
        decodedIRData.flags = IRDATA_FLAGS_IS_REPEAT | IRDATA_FLAGS_IS_LSB_FIRST;
    }
    decodedIRData.command = tCommand;
    decodedIRData.address = tAddress;
    decodedIRData.numberOfBits = SHUZU_BITS;
    decodedIRData.protocol = LG; // we have no SHUZU code

    return true;
}
