//
// MFL gd32f30x AFIO peripheral register access in C++
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

#include "gpio_config.hpp"
#include "RegRW.hpp"

namespace gpio {

class RCU;

class AFIO {
public:
    static AFIO& get_instance();

    void reset();
    void set_remap(Pin_Remap_Select remap);
    void set_exti_source(Source_Port port, Pin_Number pin);
    void set_output_event(Event_Port port, Pin_Number pin);
    void set_output_event_enable(bool enable);
    void set_compensation(bool enable);
    bool get_compensation();

    static inline constexpr uintptr_t AFIO_baseAddress = 0x40010000U;

    inline volatile uint32_t* reg_address(AFIO_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(AFIO_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    AFIO();

    // Prevent copying or assigning
    AFIO(const AFIO&) = delete;
    AFIO& operator=(const AFIO&) = delete;

    mutable bool is_clock_enabled_;
};


} // namespace gpio

extern gpio::AFIO& AFIO_I;
