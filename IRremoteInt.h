#pragma once
#ifndef IRremoteInt_h
#define IRremoteInt_h

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Raw buffer length
// 100 is sufficient for most standard protocols.
// Air-conditioner remotes often need ~750.
// Must be even (array stores space/mark pairs).
// ---------------------------------------------------------------------------
#if !defined(RAW_BUFFER_LENGTH)
#  define RAW_BUFFER_LENGTH 100  ///< Must be even. 100 supports up to 48-bit codings incl. start/stop bits.
// #define RAW_BUFFER_LENGTH 750  // Uncomment for air-conditioner remotes.
#endif
#if RAW_BUFFER_LENGTH % 2 == 1
#  error RAW_BUFFER_LENGTH must be even, since the array consists of space/mark pairs.
#endif

// ---------------------------------------------------------------------------
// Basic signal polarity constants
// ---------------------------------------------------------------------------
#define MARK   1
#define SPACE  0

// ---------------------------------------------------------------------------
// Debug / trace output
// Activate by defining DEBUG or TRACE before including this header,
// or via a compiler flag: -DDEBUG
// ---------------------------------------------------------------------------
//#define DEBUG  // lots of output from the IRremote core and all protocol decoders
//#define TRACE  // even more verbose output

#ifdef DEBUG
#  define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
#  define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#  define DEBUG_PRINT(...)   void()
#  define DEBUG_PRINTLN(...) void()
#endif

#ifdef TRACE
#  define TRACE_PRINT(...)   Serial.print(__VA_ARGS__)
#  define TRACE_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#  define TRACE_PRINT(...)   void()
#  define TRACE_PRINTLN(...) void()
#endif

// ---------------------------------------------------------------------------
// Info-level output (less verbose than DEBUG)
// ---------------------------------------------------------------------------
#ifdef INFO
#  define INFO_PRINT(...)   Serial.print(__VA_ARGS__)
#  define INFO_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#  define INFO_PRINT(...)   void()  ///< Disabled when INFO is not defined.
#  define INFO_PRINTLN(...) void()  ///< Disabled when INFO is not defined.
#endif

// ---------------------------------------------------------------------------
// LED feedback helpers
// ---------------------------------------------------------------------------
#define DISABLE_LED_FEEDBACK        false
#define ENABLE_LED_FEEDBACK         true
#define USE_DEFAULT_FEEDBACK_LED_PIN 0  ///< Use the board's built-in LED

// ---------------------------------------------------------------------------
// Compiler capability detection
// ---------------------------------------------------------------------------
#if (__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 4)
#  define COMPILER_HAS_PRAGMA_MESSAGE
#endif

#include "IRProtocol.h"

// ---------------------------------------------------------------------------
// ISR state-machine states
// ---------------------------------------------------------------------------
#define IR_REC_STATE_IDLE  0
#define IR_REC_STATE_MARK  1
#define IR_REC_STATE_SPACE 2
#define IR_REC_STATE_STOP  3  ///< Cleared back to IDLE only by resume()

// ---------------------------------------------------------------------------
// Raw buffer element type
// uint8_t saves RAM; switch to uint16_t if you need longer tick counts.
// ---------------------------------------------------------------------------
#define RAWBUF_DATA_TYPE uint8_t
#define LEARNEDNONMODULATEDTOKEN 0xFEU   ///< Used in ir_pronto
// For uint16_t mode:
// #define RAWBUF_DATA_TYPE uint16_t
// #define LEARNEDNONMODULATEDTOKEN 0x0100U

/**
 * Data and control used by the ISR and the receiver static functions.
 * Fields are ordered to minimise struct-padding on AVR.
 * Only StateForISR needs to be volatile; all others are stable while
 * data is available and before start/resume is called.
 */
struct irparams_struct {
    volatile uint8_t StateForISR;   ///< ISR state-machine state
    uint8_t          IRReceivePin;  ///< Arduino pin connected to the IR detector

#if defined(__AVR__)
    volatile uint8_t *IRReceivePinPortInputRegister;  ///< Direct port register for fast reads
    uint8_t           IRReceivePinMask;               ///< Bit mask for the pin within its port
#endif

    uint16_t TickCounterForISR;   ///< Counts MICROS_PER_TICK ticks; copied into rawbuf on each transition

    bool OverflowFlag;            ///< Set when rawbuf fills before a gap is detected

#if RAW_BUFFER_LENGTH <= 254       // saves ~75 bytes of program space and speeds up ISR
    uint8_t      rawlen;           ///< Number of entries currently in rawbuf
#else
    unsigned int rawlen;           ///< Number of entries currently in rawbuf (wide version)
#endif

    RAWBUF_DATA_TYPE rawbuf[RAW_BUFFER_LENGTH];  ///< Tick counts per mark/space; first entry is the inter-command gap
};

// ---------------------------------------------------------------------------
// Legacy compatibility (can be disabled to save ~60 bytes flash + 14 bytes RAM)
// ---------------------------------------------------------------------------
//#define NO_LEGACY_COMPATIBILITY
#if !defined(NO_LEGACY_COMPATIBILITY)
/**
 * @deprecated Use IRData / decodedIRData instead.
 * Kept for source compatibility with older sketches.
 */
struct decode_results {
    decode_type_t    decode_type;  ///< @deprecated → decodedIRData.protocol
    uint16_t         address;      ///< Used by Panasonic & Sharp (16-bit)
    uint32_t         value;        ///< @deprecated → decodedIRData.decodedRawData
    uint8_t          bits;         ///< @deprecated → decodedIRData.numberOfBits
    uint16_t         magnitude;    ///< @deprecated → decodedIRData.extra (MagiQuest)
    bool             isRepeat;     ///< @deprecated → decodedIRData.flags

    RAWBUF_DATA_TYPE *rawbuf;      ///< @deprecated → decodedIRData.rawDataPtr->rawbuf
    uint16_t          rawlen;      ///< @deprecated → decodedIRData.rawDataPtr->rawlen
    bool              overflow;    ///< @deprecated → decodedIRData.flags
};
#endif // !NO_LEGACY_COMPATIBILITY

// ---------------------------------------------------------------------------
// IRData flags (bitmask, stored in IRData::flags)
// ---------------------------------------------------------------------------
#define IRDATA_FLAGS_EMPTY              0x00  ///< No flags set
#define IRDATA_FLAGS_IS_REPEAT          0x01  ///< Frame is a repeat of the previous command
#define IRDATA_FLAGS_IS_AUTO_REPEAT     0x02  ///< Frame is an automatic repeat generated by the remote
#define IRDATA_FLAGS_PARITY_FAILED      0x04  ///< Current (auto-repeat) frame failed parity check
#define IRDATA_TOGGLE_BIT_MASK          0x08  ///< RC5/RC6 toggle bit
#define IRDATA_FLAGS_EXTRA_INFO         0x10  ///< Unexpected extra info (e.g. Kaseikyo unknown vendor ID)
#define IRDATA_FLAGS_WAS_OVERFLOW       0x40  ///< Buffer overflowed; rawlen is 0 to avoid re-triggering
#define IRDATA_FLAGS_IS_LSB_FIRST       0x00  ///< Data received LSB first (default)
#define IRDATA_FLAGS_IS_MSB_FIRST       0x80  ///< Data received MSB first (protocol-specific)

/**
 * Decoded IR data exposed to the application via IRrecv::decodedIRData.
 * Filled by protocol decoders; read by the application or print helpers.
 */
struct IRData {
    decode_type_t    protocol;       ///< Detected protocol (UNKNOWN, NEC, SONY, RC5, …)
    uint16_t         address;        ///< Decoded device address
    uint16_t         command;        ///< Decoded command/key code
    uint16_t         extra;          ///< Protocol-specific extra field (MagiQuest, Kaseikyo vendor ID, Distance ticks)
    uint16_t         numberOfBits;   ///< Bit count received (address + command + parity); useful when protocol allows variable lengths
    uint8_t          flags;          ///< See IRDATA_FLAGS_* above
    uint32_t         decodedRawData; ///< Up to 32-bit raw decoded value; used by sendRaw functions
    irparams_struct *rawDataPtr;     ///< Pointer to the raw timing buffer filled by the ISR
};

// ---------------------------------------------------------------------------
// Receiver tick resolution
// ---------------------------------------------------------------------------
#if !defined(MICROS_PER_TICK)
#  define MICROS_PER_TICK 50  ///< Microseconds per ISR tick (default 50 µs)
#endif

// ---------------------------------------------------------------------------
// Pulse-width tolerance
// ---------------------------------------------------------------------------
/** Relative tolerance in percent applied when comparing measured pulse widths. */
#define TOLERANCE 25

/** Lower tolerance multiplier (scaled to avoid floating point). */
#define LTOL (100 - TOLERANCE)
/** Upper tolerance multiplier (scaled to avoid floating point). */
#define UTOL (100 + TOLERANCE)

#if MICROS_PER_TICK == 50 && TOLERANCE == 25   // fast-path for the common defaults
#  define TICKS_LOW(us)  ((us) / 67)       // us / ((50 / 75) * 100)
#  define TICKS_HIGH(us) ((us) / 40 + 1)   // us / ((50 / 125) * 100) + 1
#else
#  define TICKS_LOW(us)  ((uint16_t)((long)(us) * LTOL / (MICROS_PER_TICK * 100)))
#  define TICKS_HIGH(us) ((uint16_t)((long)(us) * UTOL / (MICROS_PER_TICK * 100) + 1))
#endif

// ---------------------------------------------------------------------------
// Main receiver class
// ---------------------------------------------------------------------------

/**
 * Main class for receiving and decoding IR signals.
 */
class IRrecv {
public:
    struct irparams_struct irparams;  ///< Raw ISR data for this receiver instance

    IRrecv();
    IRrecv(uint8_t aReceivePin);
    IRrecv(uint8_t aReceivePin, uint8_t aFeedbackLEDPin);
    void setReceivePin(uint8_t aReceivePinNumber);

    void enableIRIn();
    void disableIRIn();

    // ------------------------------------------------------------------
    // Stream-like API
    // ------------------------------------------------------------------
    void begin(uint8_t aReceivePin, bool aEnableLEDFeedback = false,
               uint8_t aFeedbackLEDPin = USE_DEFAULT_FEEDBACK_LED_PIN);
    void start();                                      ///< Alias for enableIRIn()
    void start(uint32_t aMicrosecondsToAddToGapCounter);
    bool available();
    IRData* read();                                    ///< Returns pointer to decoded data, or nullptr
    void stop();                                       ///< Alias for disableIRIn()
    void end();

    bool isIdle();

    // ------------------------------------------------------------------
    // Core decode / resume
    // ------------------------------------------------------------------
    bool decode();   ///< Check if data is available and attempt to decode
    void resume();   ///< Enable receiving of the next value (clears STOP state)

    // ------------------------------------------------------------------
    // Print helpers
    // ------------------------------------------------------------------
    void printIRResultShort(Print *aSerial);
    void printIRResultMinimal(Print *aSerial);
    void printIRResultRawFormatted(Print *aSerial, bool aOutputMicrosecondsInsteadOfTicks = true);
    void printIRResultAsCVariables(Print *aSerial);
    void compensateAndPrintIRResultAsCArray(Print *aSerial, bool aOutputMicrosecondsInsteadOfTicks = true);
    void compensateAndPrintIRResultAsPronto(Print *aSerial, unsigned int frequency = 38000U);

    // ------------------------------------------------------------------
    // Store helpers
    // ------------------------------------------------------------------
    void   compensateAndStoreIRResultInArray(uint8_t *aArrayPtr);
    size_t compensateAndStorePronto(String *aString, unsigned int frequency = 38000U);

    // ------------------------------------------------------------------
    // Low-level decode primitives (used by individual protocol decoders)
    // ------------------------------------------------------------------
    bool decodePulseDistanceData(uint8_t aNumberOfBits, uint8_t aStartOffset,
                                 uint16_t aBitMarkMicros, uint16_t aOneSpaceMicros,
                                 uint16_t aZeroSpaceMicros, bool aMSBfirst);

    bool decodePulseWidthData(uint8_t aNumberOfBits, uint8_t aStartOffset,
                              uint16_t aOneMarkMicros, uint16_t aZeroMarkMicros,
                              uint16_t aBitSpaceMicros, bool aMSBfirst);

    bool decodeBiPhaseData(uint_fast8_t aNumberOfBits, uint_fast8_t aStartOffset,
                           uint_fast8_t aStartClockCount,
                           uint_fast8_t aValueOfSpaceToMarkTransition,
                           uint16_t aBiphaseTimeUnit);

    void    initBiphaselevel(uint8_t aRCDecodeRawbuffOffset, uint16_t aBiphaseTimeUnit);
    uint8_t getBiphaselevel();

    // ------------------------------------------------------------------
    // Protocol decoders
    // ------------------------------------------------------------------
    bool decodeBoseWave();
    bool decodeDenon();
    bool decodeJVC();
    bool decodeKaseikyo();
    bool decodeLegoPowerFunctions();
    bool decodeLG();
    bool decodeMagiQuest();   ///< Not a fully standard pulse-distance protocol
    bool decodeNEC();
    bool decodeRC5();
    bool decodeRC6();
    bool decodeSamsung();
    bool decodeSharp();       ///< Redirected to decodeDenon()
    bool decodeSony();
    bool decodeDistance();
    bool decodeHash();
    bool decodeShuzu();       ///< Template / example decoder
    bool decodeWhynter();

    // ------------------------------------------------------------------
    // Legacy / deprecated API
    // ------------------------------------------------------------------
#if !defined(NO_LEGACY_COMPATIBILITY)
    bool decodeDenonOld(decode_results *aResults);
    bool decodeJVCMSB(decode_results *aResults);
    bool decodeLGMSB(decode_results *aResults);
    bool decodeNECMSB(decode_results *aResults);
    bool decodePanasonicMSB(decode_results *aResults);
    bool decodeSonyMSB(decode_results *aResults);
    bool decodeSAMSUNG(decode_results *aResults);
    bool decodeHashOld(decode_results *aResults);

    bool decode(decode_results *aResults)
        __attribute__((deprecated("Use IrReceiver.decode() without a parameter and IrReceiver.decodedIRData.<field>.")));
#endif

    void blink13(bool aEnableLEDFeedback)
        __attribute__((deprecated("Use setLEDFeedback() or enableLEDFeedback() / disableLEDFeedback().")));

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------
    void    initDecodedIRData();
    uint8_t compare(unsigned int oldval, unsigned int newval);

    // ------------------------------------------------------------------
    // Public data members
    // ------------------------------------------------------------------
    IRData decodedIRData;           ///< Decoded IR data; read by the application after decode()

    decode_type_t lastDecodedProtocol;  ///< Protocol of the most recently decoded frame
    uint32_t      lastDecodedAddress;   ///< Address of the most recently decoded frame
    uint32_t      lastDecodedCommand;   ///< Command of the most recently decoded frame

    uint8_t repeatCount;  ///< Repeat counter used by some decoders (e.g. Denon auto-repeat)
};

extern uint8_t sBiphaseDecodeRawbuffOffset;  ///< Shared bi-phase decode state

// ---------------------------------------------------------------------------
// Mark / space matching functions
// ---------------------------------------------------------------------------
bool matchTicks(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros);
bool matchMark (uint16_t aMeasuredTicks, uint16_t aMatchValueMicros);
bool matchSpace(uint16_t aMeasuredTicks, uint16_t aMatchValueMicros);

// Legacy names (kept for backwards compatibility)
bool MATCH      (uint16_t measured, uint16_t desired);
bool MATCH_MARK (uint16_t measured_ticks, uint16_t desired_us);
bool MATCH_SPACE(uint16_t measured_ticks, uint16_t desired_us);

int  getMarkExcessMicros();
int  getMICROS_PER_TICK();   ///< Returns the active MICROS_PER_TICK value at runtime

void printIRResultShort(Print *aSerial, IRData *aIRDataPtr, uint16_t aLeadingSpaceDuration = 0);

// ---------------------------------------------------------------------------
// LED feedback
// ---------------------------------------------------------------------------
void setFeedbackLED  (bool aSwitchLedOn);
void setLEDFeedback  (uint8_t aFeedbackLEDPin, bool aEnableLEDFeedback);  ///< Pin 0 → use board LED
void setLEDFeedback  (bool aEnableLEDFeedback);  ///< Direct replacement for blink13()
void enableLEDFeedback();
void disableLEDFeedback();

void setBlinkPin(uint8_t aFeedbackLEDPin)
    __attribute__((deprecated("Use setLEDFeedback().")));

// ---------------------------------------------------------------------------
// Global singleton instances (defined in IRremote.cpp)
// ---------------------------------------------------------------------------
extern IRrecv IrReceiver;

// ---------------------------------------------------------------------------
// Main sender class
// ---------------------------------------------------------------------------

/** Duty cycle for PWM carrier. 30 % saves power and matches legacy behaviour. */
#if !defined(IR_SEND_DUTY_CYCLE)
#  define IR_SEND_DUTY_CYCLE 30
#endif

#define NO_REPEATS       0
#define SEND_STOP_BIT    true
#define SEND_REPEAT_COMMAND true  ///< For protocols where a repeat frame differs from re-sending data (e.g. NEC)

/**
 * Main class for sending IR signals.
 */
class IRsend {
public:
    IRsend();
    IRsend(uint8_t aSendPin);

    void setSendPin(uint8_t aSendPinNumber);
    void begin(uint8_t aSendPin, bool aEnableLEDFeedback = true,
               uint8_t aFeedbackLEDPin = USE_DEFAULT_FEEDBACK_LED_PIN);
    void begin(bool aEnableLEDFeedback, uint8_t aFeedbackLEDPin = USE_DEFAULT_FEEDBACK_LED_PIN)
        __attribute__((deprecated("Use begin(<sendPin>, <EnableLEDFeedback>, <LEDFeedbackPin>).")));

    size_t write(IRData *aIRSendData, uint_fast8_t aNumberOfRepeats = NO_REPEATS);

    void enableIROut(uint8_t aFrequencyKHz);

    void sendPulseDistanceWidthData(unsigned int aOneMarkMicros, unsigned int aOneSpaceMicros,
                                    unsigned int aZeroMarkMicros, unsigned int aZeroSpaceMicros,
                                    uint32_t aData, uint8_t aNumberOfBits,
                                    bool aMSBfirst, bool aSendStopBit = false);
    void sendBiphaseData(unsigned int aBiphaseTimeUnit, uint32_t aData, uint_fast8_t aNumberOfBits);

    void mark(unsigned int aMarkMicros);
    void space(unsigned int aSpaceMicros);
    void IRLedOff();

    // Raw send (tick-based)
    void sendRaw  (const uint8_t  aBufferWithTicks[],        uint_fast8_t aLengthOfBuffer, uint_fast8_t aIRFrequencyKilohertz);
    void sendRaw_P(const uint8_t  aBufferWithTicks[],        uint_fast8_t aLengthOfBuffer, uint_fast8_t aIRFrequencyKilohertz);

    // Raw send (microsecond-based)
    void sendRaw  (const uint16_t aBufferWithMicroseconds[], uint_fast8_t aLengthOfBuffer, uint_fast8_t aIRFrequencyKilohertz);
    void sendRaw_P(const uint16_t aBufferWithMicroseconds[], uint_fast8_t aLengthOfBuffer, uint_fast8_t aIRFrequencyKilohertz);

    // ------------------------------------------------------------------
    // Protocol senders (modern API)
    // ------------------------------------------------------------------
    void sendBoseWave (uint8_t aCommand, uint_fast8_t aNumberOfRepeats = NO_REPEATS);
    void sendDenon    (uint8_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats, bool aSendSharp = false);
    void sendJVC      (uint8_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats);

    void sendLGRepeat (bool aUseLG2Protocol = false);
    void sendLG       (uint8_t aAddress, uint16_t aCommand, uint_fast8_t aNumberOfRepeats,
                       bool aIsRepeat = false, bool aUseLG2Protocol = false);
    void sendLGRaw    (uint32_t aRawData, uint_fast8_t aNumberOfRepeats = 0,
                       bool aIsRepeat = false, bool aUseLG2Protocol = false);

    void sendNECRepeat();
    void sendNEC      (uint16_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats, bool aIsRepeat = false);
    void sendNECRaw   (uint32_t aRawData, uint_fast8_t aNumberOfRepeats = 0, bool aIsRepeat = false);
    void sendOnkyo    (uint16_t aAddress, uint16_t aCommand, uint_fast8_t aNumberOfRepeats, bool aIsRepeat = false);
    void sendApple    (uint8_t  aAddress, uint8_t  aCommand, uint_fast8_t aNumberOfRepeats, bool aIsRepeat = false);

    void sendKaseikyo         (uint16_t aAddress, uint8_t aData, uint_fast8_t aNumberOfRepeats, uint16_t aVendorCode);
    void sendPanasonic        (uint16_t aAddress, uint8_t aData, uint_fast8_t aNumberOfRepeats);
    void sendKaseikyo_Denon   (uint16_t aAddress, uint8_t aData, uint_fast8_t aNumberOfRepeats);
    void sendKaseikyo_Mitsubishi(uint16_t aAddress, uint8_t aData, uint_fast8_t aNumberOfRepeats);
    void sendKaseikyo_Sharp   (uint16_t aAddress, uint8_t aData, uint_fast8_t aNumberOfRepeats);
    void sendKaseikyo_JVC     (uint16_t aAddress, uint8_t aData, uint_fast8_t aNumberOfRepeats);

    void sendRC5      (uint8_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats, bool aEnableAutomaticToggle = true);
    void sendRC6      (uint8_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats, bool aEnableAutomaticToggle = true);

    void sendSamsungRepeat();
    void sendSamsung  (uint16_t aAddress, uint16_t aCommand, uint_fast8_t aNumberOfRepeats, bool aIsRepeat = false);

    void sendSharp    (uint8_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats);  ///< Redirects to sendDenon()
    void sendSony     (uint16_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats,
                       uint8_t numberOfBits = SIRCS_12_PROTOCOL);

    void sendLegoPowerFunctions(uint8_t aChannel, uint8_t tCommand, uint8_t aMode, bool aDoSend5Times = true);
    void sendLegoPowerFunctions(uint16_t aRawData, bool aDoSend5Times = true);
    void sendLegoPowerFunctions(uint16_t aRawData, uint8_t aChannel, bool aDoSend5Times = true);

    void sendMagiQuest(uint32_t wand_id, uint16_t magnitude);

    void sendPronto(const __FlashStringHelper *str,    uint_fast8_t aNumberOfRepeats = NO_REPEATS);
    void sendPronto(const char *prontoHexString,       uint_fast8_t aNumberOfRepeats = NO_REPEATS);
    void sendPronto(const RAWBUF_DATA_TYPE *data, unsigned int length, uint_fast8_t aNumberOfRepeats = NO_REPEATS);
#if defined(__AVR__)
    void sendPronto_PF(uint_farptr_t str, uint_fast8_t aNumberOfRepeats = NO_REPEATS);
    void sendPronto_P (const char *str,   uint_fast8_t aNumberOfRepeats);
#endif

    void sendShuzu(uint16_t aAddress, uint8_t aCommand, uint_fast8_t aNumberOfRepeats);  ///< Template / example sender

    // ------------------------------------------------------------------
    // Legacy / deprecated senders (MSB-first, old-style signatures)
    // ------------------------------------------------------------------
    void sendDenon (unsigned long data, int nbits);
    void sendDISH  (unsigned long data, int nbits);

    void sendJVC(unsigned long data, int nbits, bool repeat)
        __attribute__((deprecated("This old function sends MSB first! Use sendJVC(aAddress, aCommand, aNumberOfRepeats)."))) {
        sendJVCMSB(data, nbits, repeat);
    }
    void sendJVCMSB(unsigned long data, int nbits, bool repeat = false);

    void sendLG(unsigned long data, int nbits);

    void sendNEC(uint32_t aRawData, uint8_t nbits)
        __attribute__((deprecated("This old function sends MSB first! Use sendNEC(aAddress, aCommand, aNumberOfRepeats)."))) {
        sendNECMSB(aRawData, nbits);
    }
    void sendNECMSB(uint32_t data, uint8_t nbits, bool repeat = false);

    void sendPanasonic(uint16_t aAddress, uint32_t aData)
        __attribute__((deprecated("This old function sends MSB first! Use sendPanasonic(aAddress, aCommand, aNumberOfRepeats).")));

    void sendRC5(uint32_t data, uint8_t nbits);
    void sendRC5ext(uint8_t addr, uint8_t cmd, bool toggle);
    void sendRC6(uint32_t data, uint8_t nbits);
    void sendRC6(uint64_t data, uint8_t nbits);

    void sendSharpRaw(unsigned long data, int nbits);
    void sendSharp   (unsigned int address, unsigned int command);

    void sendSAMSUNG(unsigned long data, int nbits)
        __attribute__((deprecated("This old function sends MSB first! Use sendSamsung().")));

    void sendSony(unsigned long data, int nbits)
        __attribute__((deprecated("This old function sends MSB first! Use sendSony(aAddress, aCommand, aNumberOfRepeats).")));

    void sendDenonRaw(uint16_t aRawData, uint_fast8_t aNumberOfRepeats = 0)
        __attribute__((deprecated("Use sendDenon(aAddress, aCommand, aNumberOfRepeats).")));

    void sendWhynter(unsigned long data, int nbits);

    // ------------------------------------------------------------------
    // Public data / timing members
    // ------------------------------------------------------------------
    uint8_t      sendPin;
    unsigned int periodTimeMicros;     ///< Full carrier period in µs
    unsigned int periodOnTimeMicros;   ///< On-time within period, compensated for digitalWrite latency

    unsigned int getPulseCorrectionNanos();
    void         customDelayMicroseconds(unsigned long aMicroseconds);
};

extern IRsend IrSender;  ///< Global sender instance (defined in IRremote.cpp)

#endif // IRremoteInt_h
