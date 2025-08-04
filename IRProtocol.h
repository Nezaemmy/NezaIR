
#ifndef IR_PROTOCOL_H
#define IR_PROTOCOL_H

/**
 * An enum consisting of all supported formats.
 * You do NOT need to remove entries from this list when disabling protocols!
 */
typedef enum {
    UNKNOWN = 0,
    PULSE_DISTANCE,
    PULSE_WIDTH,
    DENON,
    DISH,
    JVC,
    LG,
    LG2,
    NEC,
    PANASONIC,
    KASEIKYO,
    KASEIKYO_JVC,
    KASEIKYO_DENON,
    KASEIKYO_SHARP,
    KASEIKYO_MITSUBISHI,
    RC5,
    RC6,
    SAMSUNG,
    SHARP,
    SONY,
    ONKYO,
    APPLE,
    BOSEWAVE,
    LEGO_PF,
    MAGIQUEST,
    WHYNTER,
} decode_type_t;

const __FlashStringHelper* getProtocolString(decode_type_t aProtocol);

#define PROTOCOL_IS_LSB_FIRST false
#define PROTOCOL_IS_MSB_FIRST true

/*
 * Carrier frequencies for various protocols
 */
#define SONY_KHZ        40
#define BOSEWAVE_KHZ    38
#define DENON_KHZ       38
#define JVC_KHZ         38
#define LG_KHZ          38
#define NEC_KHZ         38
#define SAMSUNG_KHZ     38
#define KASEIKYO_KHZ    37
#define RC5_RC6_KHZ     36

/*
 * Constants for some protocols
 */
#define PANASONIC_VENDOR_ID_CODE    0x2002
#define DENON_VENDOR_ID_CODE        0x3254
#define MITSUBISHI_VENDOR_ID_CODE   0xCB23
#define SHARP_VENDOR_ID_CODE        0x5AAA
#define JVC_VENDOR_ID_CODE          0x0103

#define SIRCS_12_PROTOCOL       12
#define SIRCS_15_PROTOCOL       15
#define SIRCS_20_PROTOCOL       20

#define LEGO_MODE_EXTENDED  0
#define LEGO_MODE_COMBO     1
#define LEGO_MODE_SINGLE    0x4 // here the 2 LSB have meanings like Output A / Output B

#endif // IR_PROTOCOL_H

#pragma once
