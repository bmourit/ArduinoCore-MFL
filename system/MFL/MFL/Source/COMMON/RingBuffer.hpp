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

#include <cstdint>
#include <cstddef>

template <typename T, size_t N>
class RingBuffer {
public:
    RingBuffer() = default;

    // Write a single element to the buffer, returns true if successful
    inline auto write(const T& value) -> bool {
        if (isFull()) return false;  // Prevent overflow
        buffer_[head_] = value;
        head_ = (head_ + 1 == N) ? 0 : head_ + 1;
        full_ = (head_ == tail_);
        return true;
    }

    // Read a single element from the buffer, returns true if successful
    inline auto read(T& value) -> bool {
        if (isEmpty()) return false;  // No data to read
        value = buffer_[tail_];
        tail_ = (tail_ + 1 == N) ? 0 : tail_ +1;
        full_ = false;
        return true;
    }

    // Peek the next element without removing it, returns true if valid
    inline auto peek(T& value) const -> bool {
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
    [[nodiscard]] inline auto isEmpty() const -> bool { return (!full_ && (head_ == tail_)); }

    // Check if the buffer is full
    [[nodiscard]] inline auto isFull() const -> bool { return full_; }

    // Get the current number of elements available for reading
    [[nodiscard]] inline auto availableForRead() const -> size_t { return size(); }

    // Get the number of free slots available for writing
    [[nodiscard]] inline auto availableForWrite() const -> size_t { return N - size(); }

    // Get the maximum capacity of the buffer
    [[nodiscard]] inline constexpr auto capacity() const -> size_t { return N; }

    // Provides r/w access to the buffer head
    inline auto getHead() -> size_t { return head_; }
    inline void setHead(size_t value) { head_ = value; }

    // Provides r/w access to the buffer tail
    inline auto getTail() -> size_t { return tail_; }
    inline  void setTail(size_t value) { tail_ = value; }

    // Provides access to the underlying buffer
    inline auto data() -> T* { return buffer_; }

    // Provides a const version for read-only access
    inline auto data() const -> const T* { return buffer_; }

private:
    T buffer_[N];           // Circular buffer storage
    volatile size_t head_{0};  // Head index (next write position)
    volatile size_t tail_{0};  // Tail index (next read position)
    volatile bool full_{false};    // Indicates if the buffer is full

    // Get the current number of elements in the buffer
    [[nodiscard]] inline auto size() const -> size_t {
        if (full_) return N;
        return (head_ >= tail_) ? (head_ - tail_) : (N + head_ - tail_);
    }
};
