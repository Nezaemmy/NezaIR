#pragma once
#ifndef CPPLIST_H
#define CPPLIST_H

#include <Arduino.h>
#include "IRremoteInt.h"

#ifndef IR_MAXCOUNTIR
#define IR_MAXCOUNTIR 8
#endif

class CppList {
public:
    CppList() {
        for (uint8_t i = 0; i < IR_MAXCOUNTIR; ++i) {
            _items[i] = nullptr;
        }
    }

    bool Add(IRrecv *item) {
        if (item == nullptr) return false;

        for (uint8_t i = 0; i < IR_MAXCOUNTIR; ++i) {
            if (_items[i] == item) return true;
        }

        for (uint8_t i = 0; i < IR_MAXCOUNTIR; ++i) {
            if (_items[i] == nullptr) {
                _items[i] = item;
                return true;
            }
        }
        return false;
    }

    bool Remove(IRrecv *item) {
        if (item == nullptr) return false;

        for (uint8_t i = 0; i < IR_MAXCOUNTIR; ++i) {
            if (_items[i] == item) {
                _items[i] = nullptr;
                return true;
            }
        }
        return false;
    }

    IRrecv *GetItem(uint8_t index) const {
        if (index >= IR_MAXCOUNTIR) return nullptr;
        return _items[index];
    }

    constexpr uint8_t Capacity() const {
        return IR_MAXCOUNTIR;
    }

private:
    IRrecv *_items[IR_MAXCOUNTIR];
};

#endif
