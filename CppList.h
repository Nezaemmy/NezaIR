#pragma once
#ifndef CPPLIST_H
#define CPPLIST_H

#include <Arduino.h>
#include <string.h>
#include "IRremoteInt.h"

#ifndef IR_MAXCOUNTIR
#define IR_MAXCOUNTIR 8
#endif

class CppList {
public:
    CppList() : _count(0) {
        memset(_arr, 0, sizeof(_arr));
    }

    void Add(irparams_struct *item) {
        if (item == nullptr) return;
        if (Exists(item)) return;
        if (_count >= IR_MAXCOUNTIR) return;

        _arr[_count++] = item;
    }

    bool Remove(irparams_struct *item) {
        int idx = GetIndex(item);
        if (idx < 0) return false;

        for (int i = idx; i < _count - 1; ++i) {
            _arr[i] = _arr[i + 1];
        }

        _arr[--_count] = nullptr;
        return true;
    }

    int GetCount() const {
        return _count;
    }

    irparams_struct *GetItem(int index) const {
        if (index < 0 || index >= _count) return nullptr;
        return _arr[index];
    }

    bool IsEmpty() const {
        return _count == 0;
    }

    bool IsFull() const {
        return _count >= IR_MAXCOUNTIR;
    }

private:
    bool Exists(const irparams_struct *item) const {
        return GetIndex(item) >= 0;
    }

    int GetIndex(const irparams_struct *item) const {
        for (int i = 0; i < _count; ++i) {
            if (_arr[i] == item) return i;
        }
        return -1;
    }

private:
    int _count;
    irparams_struct *_arr[IR_MAXCOUNTIR];
};

#endif // CPPLIST_H
