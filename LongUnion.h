

#ifndef LONG_UNION_H
#define LONG_UNION_H

#include <Arduino.h>
#include <stdint.h>

/**
 * Union to specify parts / manifestations of a 16 bit Word without casts and shifts.
 * It also supports the compiler generating small code.
 */
union WordUnion {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } UByte;
    struct {
        int8_t LowByte;
        int8_t HighByte;
    } Byte;
    uint8_t UBytes[2];
    int8_t Bytes[2];
    uint16_t UWord;
    int16_t Word;
    uint8_t * BytePointer;
};

/**
 * Union to specify parts / manifestations of a 32 bit Long without casts and shifts.
 * It also supports the compiler generating small code.
 */
union LongUnion {
    struct {
        uint8_t LowByte;
        uint8_t MidLowByte;
        uint8_t MidHighByte;
        uint8_t HighByte;
    } UByte;
    struct {
        int8_t LowByte;
        int8_t MidLowByte;
        int8_t MidHighByte;
        int8_t HighByte;
    } Byte;
    struct {
        uint8_t LowByte;
        WordUnion MidWord;
        uint8_t HighByte;
    } ByteWord;
    struct {
        int16_t LowWord;
        int16_t HighWord;
    } Word;
    struct {
        WordUnion LowWord;
        WordUnion HighWord;
    } WordUnion;
    struct {
        uint16_t LowWord;
        uint16_t HighWord;
    } UWord;
    uint8_t UBytes[4];
    int8_t Bytes[4];
    uint16_t UWords[2];
    int16_t Words[2];
    uint32_t ULong;
    int32_t Long;
};

#endif // LONG_UNION_H

#pragma once
