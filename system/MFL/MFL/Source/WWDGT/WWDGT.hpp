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

#include <stdint.h>

#include "wwdgt_config.hpp"
#include "RegRW.hpp"

namespace wwdgt {

class RCU;

class WWDGT {
public:
    static WWDGT& get_instance();

    void reset();
    void enable();
    void update_counter(uint16_t value);
    void setup(uint16_t value, uint16_t window, Prescaler_Values prescaler);
    bool get_flag();
    void clear_flag();
    void set_interrupt_enable(bool enable);

    static inline constexpr uintptr_t WWDGT_baseAddress = 0x40002C00U;

    inline volatile uint32_t* reg_address(WWDGT_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(WWDGT_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    WWDGT();

    mutable bool is_clock_enabled_;
};

} // namespace wwdgt
