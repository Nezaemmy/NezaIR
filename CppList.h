#ifndef CPPLIST_H
#define CPPLIST_H

#include "IRremoteInt.h"

#ifndef IR_MAXCOUNTIR
#define IR_MAXCOUNTIR 8
#endif

class CppList {
public:
    CppList() : _count(0) {}

    inline void Add(irparams_struct *item) {
        if (item == nullptr) return;
        if (Exists(item)) return;
        if (_count >= IR_MAXCOUNTIR) return;
        _arr[_count++] = item;
    }

    inline int GetCount() const {
        return _count;
    }

    inline irparams_struct* GetItem(int index) const {
        if (index < 0) return nullptr;
        if (index >= _count) return nullptr;
        return _arr[index];
    }

private:
    inline bool Exists(const void *item) const {
        return GetIndex(item) != -1;
    }

    inline int GetIndex(const void *item) const {
        for (int i = 0; i < _count; ++i) {
            if (item == _arr[i]) return i;
        }
        return -1;
    }

    int _count;
    irparams_struct* _arr[IR_MAXCOUNTIR];
};

#endif // CPPLIST_H
#pragma once
