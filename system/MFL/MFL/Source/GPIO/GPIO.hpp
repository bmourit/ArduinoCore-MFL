//
// MFL gd32f30x GPIO peripheral register access in C++
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

#include "gpio_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"

namespace gpio {

class GPIO {
public:
    static auto get_instance(GPIO_Base Base) -> Result<GPIO, GPIO_Error_Type>;

    // Reset
    void reset();

    // Set and get pin mode
    void set_pin_mode(Pin_Number pin, Pin_Mode mode, Output_Speed speed = Output_Speed::SPEED_50MHZ);
    auto get_pin_mode(Pin_Number pin) -> Pin_Mode;

    // Pin level
    void set_pin_high(Pin_Number pin);
    void set_pin_low(Pin_Number pin);
    void set_pin_level(Pin_Number pin, bool high);

    // Read/write
    void write_pin(Pin_Number pin, bool set);
    auto read_pin(Pin_Number pin) -> bool;
    void toggle_pin(Pin_Number pin);

    // Port
    void set_port(uint16_t data);

    // Pin state
    auto get_pin_input_state(Pin_Number pin) -> bool;
    auto get_pin_output_state(Pin_Number pin) -> bool;

    // Port state
    auto get_port_input_state() -> uint16_t;
    auto get_port_output_state() -> uint16_t;

    // Lock
    void lock_pin(Pin_Number pin);

    // Accessor methods
    inline auto get_base() -> GPIO_Base { return base_; }

    // Register address
    [[nodiscard]] inline auto reg_address(GPIO_Regs reg) const -> volatile uint32_t* {
        const auto idx = static_cast<uint32_t>(base_);
        return reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[idx] + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(GPIO_Base::INVALID)> clock_enabled_;
    explicit GPIO(GPIO_Base Base);

    GPIO_Base base_;
    GPIO_Clock_Config GPIO_pclk_info_;

    template <GPIO_Base Base>
    friend auto get_instance_for_base() -> GPIO&;
};

[[gnu::always_inline]] static inline void fast_set_pin(GPIO_Base base, Pin_Number pin) {
    volatile uint32_t* bop_data = reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[static_cast<size_t>(base)] + BOP_OFFSET);
    *bop_data = (1U << static_cast<uint32_t>(pin));
}

[[gnu::always_inline]] static inline void fast_clear_pin(GPIO_Base base, Pin_Number pin) {
    volatile uint32_t* bc_data = reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[static_cast<size_t>(base)] + BC_OFFSET);
    *bc_data = (1U << static_cast<uint32_t>(pin));
}

[[gnu::always_inline]] static inline void fast_write_pin(GPIO_Base base, Pin_Number pin, bool value) {
    if (value)
        fast_set_pin(base, pin);
    else
        fast_clear_pin(base, pin);
}

[[gnu::always_inline]] static inline auto fast_read_pin(GPIO_Base base, Pin_Number pin) -> bool {
    volatile auto* istat_data = reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[static_cast<size_t>(base)] + ISTAT_OFFSET);
    return (*istat_data & (1U << static_cast<uint32_t>(pin))) != Clear;
}

[[gnu::always_inline]] static inline void fast_toggle_pin(GPIO_Base base, Pin_Number pin) {
    volatile auto* octl_data = reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[static_cast<size_t>(base)] + OCTL_OFFSET);
    bool value = (*octl_data & (1U << static_cast<uint32_t>(pin))) ? false : true;
    fast_write_pin(base, pin, value);
}

} // namespace gpio
