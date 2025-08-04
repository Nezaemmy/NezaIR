
#include <Arduino.h>

//#define DEBUG // Activate this for lots of lovely debug output from this decoder.
#include "IRremoteInt.h" // evaluates the DEBUG for DEBUG_PRINT



// MSB first, 1 start bit + 4 bit channel, 4 bit mode + 4 bit command + 4 bit parity + 1 stop bit.
#define LEGO_CHANNEL_BITS       4
#define LEGO_MODE_BITS          4
#define LEGO_COMMAND_BITS       4
#define LEGO_PARITY_BITS        4

#define LEGO_BITS               (LEGO_CHANNEL_BITS + LEGO_MODE_BITS + LEGO_COMMAND_BITS + LEGO_PARITY_BITS)

#define LEGO_HEADER_MARK        158    //  6 cycles
#define LEGO_HEADER_SPACE       1026   // 39 cycles

#define LEGO_BIT_MARK           158    //  6 cycles
#define LEGO_ONE_SPACE          553    // 21 cycles
#define LEGO_ZERO_SPACE         263    // 10 cycles

#define LEGO_AVERAGE_DURATION   11000 // LEGO_HEADER_MARK + LEGO_HEADER_SPACE  + 16 * 600 + 158

#define LEGO_AUTO_REPEAT_PERIOD_MIN 110000 // Every frame is auto repeated 5 times.
#define LEGO_AUTO_REPEAT_PERIOD_MAX 230000 // space for channel 3

/*
 * Compatibility function for legacy code, this calls the send raw data function
 */
void IRsend::sendLegoPowerFunctions(uint16_t aRawData, bool aDoSend5Times) {
    sendLegoPowerFunctions(aRawData, (aRawData >> (LEGO_MODE_BITS + LEGO_COMMAND_BITS + LEGO_PARITY_BITS)) & 0x3, aDoSend5Times);
}

/*
 * Here we process the structured data, and call the send raw data function
 */
void IRsend::sendLegoPowerFunctions(uint8_t aChannel, uint8_t aCommand, uint8_t aMode, bool aDoSend5Times) {
    aChannel &= 0x0F; // allow toggle and escape bits too
    aCommand &= 0x0F;
    aMode &= 0x0F;
    uint8_t tParity = 0xF ^ aChannel ^ aMode ^ aCommand;
    // send 4 bit channel, 4 bit mode, 4 bit command, 4 bit parity
    uint16_t tRawData = (((aChannel << LEGO_MODE_BITS) | aMode) << (LEGO_COMMAND_BITS + LEGO_PARITY_BITS))
            | (aCommand << LEGO_PARITY_BITS) | tParity;
    sendLegoPowerFunctions(tRawData, aChannel, aDoSend5Times);
}

void IRsend::sendLegoPowerFunctions(uint16_t aRawData, uint8_t aChannel, bool aDoSend5Times) {
    enableIROut(38);

    DEBUG_PRINT("sendLego aRawData=0x");
    DEBUG_PRINTLN(aRawData, HEX);

    aChannel &= 0x03; // we have 4 channels

    uint_fast8_t tNumberOfCommands = 1;
    if (aDoSend5Times) {
        tNumberOfCommands = 5;
    }
// required for repeat timing, see http://www.hackvandedam.nl/blog/?page_id=559
    uint8_t tRepeatPeriod = (110 - (LEGO_AVERAGE_DURATION / 1000)) + (aChannel * 40); // from 100 to 220

    while (tNumberOfCommands > 0) {

        // Header
        mark(LEGO_HEADER_MARK);
        space(LEGO_HEADER_SPACE);

        sendPulseDistanceWidthData(LEGO_BIT_MARK, LEGO_ONE_SPACE, LEGO_BIT_MARK, LEGO_ZERO_SPACE, aRawData, LEGO_BITS, PROTOCOL_IS_MSB_FIRST,
        SEND_STOP_BIT);

        tNumberOfCommands--;
        // skip last delay!
        if (tNumberOfCommands > 0) {
            // send repeated command with a fixed space gap
            delay(tRepeatPeriod);
        }
    }
}

/*
 * Mode is stored in the upper nibble of command
 */
bool IRrecv::decodeLegoPowerFunctions() {

    // Check header "mark"
    if (!matchMark(decodedIRData.rawDataPtr->rawbuf[1], LEGO_HEADER_MARK)) {
        // no debug output, since this check is mainly to determine the received protocol
        return false;
    }

    // Check we have enough data - +4 for initial gap, start bit mark and space + stop bit mark
    if (decodedIRData.rawDataPtr->rawlen != (2 * LEGO_BITS) + 4) {
        DEBUG_PRINT("LEGO: ");
        DEBUG_PRINT("Data length=");
        DEBUG_PRINT(decodedIRData.rawDataPtr->rawlen);
        DEBUG_PRINTLN(" is not 36");
        return false;
    }
    // Check header "space"
    if (!matchSpace(decodedIRData.rawDataPtr->rawbuf[2], LEGO_HEADER_SPACE)) {
        DEBUG_PRINT("LEGO: ");
        DEBUG_PRINTLN("Header space length is wrong");
        return false;
    }

    if (!decodePulseDistanceData(LEGO_BITS, 3, LEGO_BIT_MARK, LEGO_ONE_SPACE, LEGO_ZERO_SPACE, PROTOCOL_IS_MSB_FIRST)) {
        DEBUG_PRINT("LEGO: ");
        DEBUG_PRINTLN("Decode failed");
        return false;
    }

    // Stop bit
    if (!matchMark(decodedIRData.rawDataPtr->rawbuf[3 + (2 * LEGO_BITS)], LEGO_BIT_MARK)) {
        DEBUG_PRINT("LEGO: ");
        DEBUG_PRINTLN(F("Stop bit mark length is wrong"));
        return false;
    }

    // Success
    decodedIRData.flags = IRDATA_FLAGS_IS_MSB_FIRST;
    uint16_t tDecodedValue = decodedIRData.decodedRawData;
    uint8_t tToggleEscapeChannel = tDecodedValue >> (LEGO_MODE_BITS + LEGO_COMMAND_BITS + LEGO_PARITY_BITS);
    uint8_t tMode = (tDecodedValue >> (LEGO_COMMAND_BITS + LEGO_PARITY_BITS)) & 0xF;
    uint8_t tData = (tDecodedValue >> LEGO_PARITY_BITS) & 0xF; // lego calls this field "data"
    uint8_t tParityReceived = tDecodedValue & 0xF;

    // This is parity as defined in the specifications
    // But in some scans I saw 0x9 ^ .. as parity formula
    uint8_t tParityComputed = 0xF ^ tToggleEscapeChannel ^ tMode ^ tData;

    // parity check
    if (tParityReceived != tParityComputed) {
        DEBUG_PRINT("LEGO: ");
        DEBUG_PRINT("Parity is not correct. expected=0x");
        DEBUG_PRINT(tParityComputed, HEX);
        DEBUG_PRINT(" received=0x");
        DEBUG_PRINT(tParityReceived, HEX);
        DEBUG_PRINT(", raw=0x");
        DEBUG_PRINT(tDecodedValue, HEX);
        DEBUG_PRINT(", 3 nibbles are 0x");
        DEBUG_PRINT(tToggleEscapeChannel, HEX);
        DEBUG_PRINT(", 0x");
        DEBUG_PRINT(tMode, HEX);
        DEBUG_PRINT(", 0x");
        DEBUG_PRINTLN(tData, HEX);
        // might not be an error, so just continue
        decodedIRData.flags = IRDATA_FLAGS_PARITY_FAILED | IRDATA_FLAGS_IS_MSB_FIRST;
    }

    /*
     * Check for autorepeat (should happen 4 times for one press)
     */
    int iMICROS_PER_TICK = getMICROS_PER_TICK(); //Modified to get the correct iMICROS_PER_TICK
    if (decodedIRData.rawDataPtr->rawbuf[0] < (LEGO_AUTO_REPEAT_PERIOD_MAX / iMICROS_PER_TICK)) {
        decodedIRData.flags |= IRDATA_FLAGS_IS_AUTO_REPEAT;
    }
    decodedIRData.address = tToggleEscapeChannel;
    decodedIRData.command = tData | tMode << LEGO_COMMAND_BITS;
    decodedIRData.protocol = LEGO_PF;
    decodedIRData.numberOfBits = LEGO_BITS;

    return true;
}

/** @}*/
