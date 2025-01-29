//
// MFL gd32f30x CORTEX peripheral register access in C++
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

#include "cortex_config.hpp"

namespace cortex {

class CORTEX {
public:
    static CORTEX& get_instance();

    void set_low_power_mode_enable(Low_Power_Mode mode, bool enable);
    // SysTick
    void set_systick_source(Systick_Source source);
    // Vector table
    void set_nvic_vector_table(Vector_Table base, uint32_t offset);
    // Priority
    void set_nvic_priority_group(Priority_Group group);
    void set_nvic_priority(uint8_t irq, uint8_t preemption_priority, uint8_t sub_priority);
    // Interrupts
    void nvic_irq_enable(uint8_t irq);
    void nvic_irq_disable(uint8_t irq);


    inline bool get_systick_active_flag() {
        return ((SysTick->CTRL & SysTickCountFlagMask) == (SysTickCountFlagMask));
    }

private:
    CORTEX();

    // Prevent copying or assigning
    CORTEX(const CORTEX&) = delete;
    CORTEX& operator=(const CORTEX&) = delete;
};


} // namespace cortex

extern cortex::CORTEX& CORTEX_I;
