//
// MFL gd32f30x CRC functionality in C++
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

#include "crc_config.hpp"
#include "RegRW.hpp"

namespace crc {

class RCU;

class CRC {
public:
    static CRC& get_instance();

    // Reset
    void reset();
    // Data
    void reset_data();
    uint32_t get_data();
    // Free data
    uint8_t get_free_data();
    void set_free_data(uint8_t data);
    // Calculate
    uint32_t calculate_data(uint32_t data);
    uint32_t calculate_mulitple_data(const uint32_t* array, uint32_t size);

    // Base address
    static inline constexpr uintptr_t CRC_baseAddress = 0x40023000U;

    // Register address
    inline volatile uint32_t* reg_address(CRC_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(CRC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    CRC();

    // Prevent copying or assigning
    CRC(const CRC&) = delete;
    CRC& operator=(const CRC&) = delete;

    mutable bool is_clock_enabled_;
};

} // namespace crc
