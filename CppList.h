#pragma once
#ifndef CPPLIST_H
#define CPPLIST_H

#include <string.h>      // memset
#include "IRremoteInt.h"

#ifndef IR_MAXCOUNTIR
#define IR_MAXCOUNTIR 8
#endif

class CppList {
public:
    CppList() : _count(0) {
        // Zero out the array so unoccupied slots are never garbage pointers.
        // Safe to inspect in a debugger even before any Add() call.
        memset(_arr, 0, sizeof(_arr));
    }

    // Add a receiver. Silently ignores nullptr, duplicates, and a full list.
    void Add(irparams_struct *item) {
        if (item == nullptr)          return;
        if (Exists(item))             return;
        if (_count >= IR_MAXCOUNTIR)  return;
        _arr[_count++] = item;
    }

    // Remove a receiver and compact the array.
    // Returns true if the item was found and removed, false otherwise.
    bool Remove(irparams_struct *item) {
        int idx = GetIndex(item);
        if (idx == -1) return false;

        // Shift everything after idx one position left
        for (int i = idx; i < _count - 1; ++i) {
            _arr[i] = _arr[i + 1];
        }
        _arr[--_count] = nullptr;   // clear the now-vacant tail slot
        return true;
    }

    // Number of registered receivers.
    int GetCount() const {
        return _count;
    }

    // Bounds-checked element access. Returns nullptr for out-of-range index.
    irparams_struct* GetItem(int index) const {
        if (index < 0 || index >= _count) return nullptr;
        return _arr[index];
    }

    // Returns true when the list has no receivers.
    bool IsEmpty() const {
        return _count == 0;
    }

    // Returns true when the list is at maximum capacity.
    bool IsFull() const {
        return _count >= IR_MAXCOUNTIR;
    }

private:
    bool Exists(const irparams_struct *item) const {
        return GetIndex(item) != -1;
    }

    // Returns the index of item, or -1 if not present.
    int GetIndex(const irparams_struct *item) const {
        for (int i = 0; i < _count; ++i) {
            if (_arr[i] == item) return i;
        }
        return -1;
    }

    int              _count;
    irparams_struct* _arr[IR_MAXCOUNTIR];
};

#endif // CPPLIST_H
