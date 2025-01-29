//
// MFL gd32f30x DBG debug in C++
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

#include "dbg_config.hpp"
#include "RegRW.hpp"

namespace armdbg {

class RCU;

class ARMDBG {
public:
    ARMDBG& get_instance();

    // Reset
    void reset();
    // Debug ID
    uint32_t get_debug_id();
    // Trace
    void set_debug_trace_enable(bool enable);
    // Peripheral
    void set_peripheral_debug_enable(Debug_Peripheral peripheral, bool enable);
    // Low power
    void set_debug_low_power_enable(Low_Power_Debug type, bool enable);

    // Base address
    static inline constexpr uintptr_t DBG_baseAddress = 0xE0042000U;

    // Offset access
    inline volatile uint32_t* reg_address(DBG_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(DBG_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    ARMDBG();

    // Prevent copying or assigning
    ARMDBG(const ARMDBG&) = delete;
    ARMDBG& operator=(const ARMDBG&) = delete;
};


} // namespace armdbg
