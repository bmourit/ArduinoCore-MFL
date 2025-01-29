//
// MFL EXTI peripheral register access in C++
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

#include "exti_config.hpp"
#include "RegRW.hpp"

namespace exti {

class RCU;

class EXTI {
public:
    static EXTI& get_instance();

    // Initialize
    void init(EXTI_Line line, EXTI_Mode mode, EXTI_Trigger trigger);
    // Reset
    void reset();
    // Events
    void set_event_enable(EXTI_Line line, bool enable);
    // Interrupts and flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Status_Flags flag);
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Interrupt_Flags flag);
    void set_interrupt_enable(Interrupt_Type type, bool enable);
    void set_software_interrupt_enable(EXTI_Line line, bool enable);

    static inline constexpr uintptr_t EXTI_baseAddress = 0x40010400U;

    inline volatile uint32_t* reg_address(EXTI_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(EXTI_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    EXTI();

    // Prevent copying or assigning
    EXTI(const EXTI&) = delete;
    EXTI& operator=(const EXTI&) = delete;
};

} // namespace exti
