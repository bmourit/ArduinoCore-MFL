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

#include <cstdint>
#include <array>

#include "crc_config.hpp"
#include "RegRW.hpp"

namespace crc {

class CRC {
public:
    static auto get_instance() -> CRC&;

    // Reset
    void reset();

    // Data
    void reset_data();
    auto get_data() -> uint32_t;

    // Free data
    auto get_free_data() -> uint8_t;
    void set_free_data(uint8_t data);

    // Calculate
    auto calculate_data(uint32_t data) -> uint32_t;

    // Calculate array
    template <size_t N>
    inline auto calculate_multiple_data(const std::array<uint32_t, N>&data) -> uint32_t {
        for (size_t i = 0; i < N; ++i) {
            write_register(*this, CRC_Regs::DATA, data[i]);
        }
        return read_register<uint32_t>(*this, CRC_Regs::DATA);
    }

    // Using c-style array
    template <size_t N>
    inline auto calculate_multiple_data(const uint32_t (&data)[N]) -> uint32_t {
        for (size_t i = 0; i < N; ++i) {
            write_register(*this, CRC_Regs::DATA, data[i]);
        }
        return read_register<uint32_t>(*this, CRC_Regs::DATA);
    }

    // Base address
    static inline constexpr uintptr_t CRC_baseAddress = 0x4002'3000U;

    // Register address
    [[nodiscard]] inline auto reg_address(CRC_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(CRC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    CRC();

    // Prevent copying or assigning
    CRC(const CRC&) = delete;
    auto operator=(const CRC&) -> CRC& = delete;

    mutable bool is_clock_enabled_;
};

} // namespace crc
