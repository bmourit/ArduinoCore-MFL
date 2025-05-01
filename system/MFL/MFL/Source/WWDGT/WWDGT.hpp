//
// MFL gd32f30x WWDGT peripheral register access in C++
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

#include "wwdgt_config.hpp"
#include "RegRW.hpp"

namespace wwdgt {

class WWDGT {
public:
    static auto get_instance() -> WWDGT&;

    // Reset
    void reset();

    // Enable
    void enable();

    // Counter
    void update_counter(uint16_t value);

    // Configuration
    void setup(uint16_t value, uint16_t window, Prescaler_Values prescaler);

    // Interrupts and flags
    auto get_flag() -> bool;
    void clear_flag();
    void set_interrupt_enable(bool enable);

    static inline constexpr uintptr_t WWDGT_baseAddress = 0x4000'2C00U;

    [[nodiscard]] inline auto reg_address(WWDGT_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(WWDGT_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    WWDGT();

    // Prevent copying or assigning
    WWDGT(const WWDGT&) = delete;
    auto operator=(const WWDGT&) -> WWDGT& = delete;

    mutable bool is_clock_enabled_;
};

} // namespace wwdgt
