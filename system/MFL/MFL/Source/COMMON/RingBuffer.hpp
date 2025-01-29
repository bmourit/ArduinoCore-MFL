//
// MFL Template RingBuffer
//
// Copyright (C) 2025 B. Mouritsen <bnmguy@gmail.com>. All rights reserved.
//
// This file is part of the Microcontroller Firmware Library (MFL).
//
// MFL is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// MFL is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with MFL.
// If not, see <https://www.gnu.org/licenses/>.
//

#pragma once

#include <stdint.h>
#include <stddef.h>

template <typename T, size_t N>
class RingBuffer {
public:
    RingBuffer() : head_(0), tail_(0), full_(false) {}

    // Write a single element to the buffer, returns true if successful
    inline bool write(const T& value) {
        if (isFull()) return false;  // Prevent overflow
        buffer_[head_] = value;
        head_ = (head_ + 1) % N;
        full_ = (head_ == tail_);
        return true;
    }

    // Read a single element from the buffer, returns true if successful
    inline bool read(T& value) {
        if (isEmpty()) return false;  // No data to read
        value = buffer_[tail_];
        tail_ = (tail_ + 1) % N;
        full_ = false;
        return true;
    }

    // Peek the next element without removing it, returns true if valid
    inline bool peek(T& value) const {
        if (isEmpty()) return false;
        value = buffer_[tail_];
        return true;
    }

    // Flush the buffer (clear all data)
    inline void flush() {
        head_ = 0;
        tail_ = 0;
        full_ = false;
    }

    // Check if the buffer is empty
    inline bool isEmpty() const { return (!full_ && (head_ == tail_)); }

    // Check if the buffer is full
    inline bool isFull() const { return full_; }

    // Get the current number of elements available for reading
    inline size_t availableForRead() const { return size(); }

    // Get the number of free slots available for writing
    inline size_t availableForWrite() const { return N - size(); }

    // Get the maximum capacity of the buffer
    inline constexpr size_t capacity() const { return N; }

    // Provides r/w access to the buffer head
    inline size_t getHead() { return head_; }
    inline void setHead(size_t value) {
        head_ = value;
    }

    // Provides r/w access to the buffer tail
    inline size_t getTail() { return tail_; }
    inline  void setTail(size_t value) {
        tail_ = value;
    }

    // Provides access to the underlying buffer
    inline T* data() { return buffer_; }

    // Provides a const version for read-only access
    inline const T* data() const { return buffer_; }

private:
    T buffer_[N];           // Circular buffer storage
    volatile size_t head_;  // Head index (next write position)
    volatile size_t tail_;  // Tail index (next read position)
    volatile bool full_;    // Indicates if the buffer is full

    // Get the current number of elements in the buffer
    inline size_t size() const {
        if (full_) return N;
        return (head_ >= tail_) ? (head_ - tail_) : (N + head_ - tail_);
    }
};
