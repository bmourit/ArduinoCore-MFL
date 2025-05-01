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

#include <cstdint>

#include "gpio_config.hpp"
#include "RegRW.hpp"

namespace gpio {

class AFIO {
public:
    static auto get_instance() -> AFIO&;

    // Reset
    void reset();

    // Remap
    void set_remap(Pin_Remap_Select remap);

    // EXTI
    void set_exti_source(Source_Port port, Pin_Number pin);

    // Event
    void set_output_event(Event_Port port, Pin_Number pin);
    void set_output_event_enable(bool enable);

    // Compensation
    void set_compensation(bool enable);
    auto get_compensation() -> bool;

    static inline constexpr uintptr_t AFIO_baseAddress = 0x4001'0000U;

    [[nodiscard]] inline auto reg_address(AFIO_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(AFIO_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    AFIO();

    // Prevent copying or assigning
    AFIO(const AFIO&) = delete;
    auto operator=(const AFIO&) -> AFIO& = delete;

    mutable bool is_clock_enabled_;
};

} // namespace gpio

extern gpio::AFIO& AFIO_I;
