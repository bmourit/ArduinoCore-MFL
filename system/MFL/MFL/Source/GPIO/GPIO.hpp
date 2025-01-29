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

#include <stdint.h>
#include <array>

#include "gpio_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"

namespace gpio {

class RCU;
class AFIO;

class GPIO {
public:
    static Result<GPIO, GPIO_Error_Type> get_instance(GPIO_Base Base);

    // Reset
    void reset();
    // Set and get pin mode
    void set_pin_mode(Pin_Number pin, Pin_Mode mode, Output_Speed speed = Output_Speed::SPEED_MAX);
    Pin_Mode get_pin_mode(Pin_Number pin);
    // Pin level
    void set_pin_high(Pin_Number pin);
    void set_pin_low(Pin_Number pin);
    void set_pin_level(Pin_Number pin, bool high);
    // Read/write
    void write_pin(Pin_Number pin, bool set);
    bool read_pin(Pin_Number pin);
    void toggle_pin(Pin_Number pin);
    // Port
    void set_port(uint16_t data);
    // Pin state
    bool get_pin_input_state(Pin_Number pin);
    bool get_pin_output_state(Pin_Number pin);
    // Port state
    uint16_t get_port_input_state();
    uint16_t get_port_output_state();
    // Lock
    void lock_pin(Pin_Number pin);

    // Accessor methods
    GPIO_Base get_base() { return base_; }

    // Register address
    inline volatile uint32_t* reg_address(GPIO_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(base_address_ + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(GPIO_Base::INVALID)> clock_enabled_;
    explicit GPIO(GPIO_Base Base);

    GPIO_Base base_;
    GPIO_Clock_Config GPIO_pclk_info_;
    uint32_t base_address_;

    template <GPIO_Base Base>
    friend GPIO& get_instance_for_base();
};

static inline void fast_set_pin(GPIO_Base base, Pin_Number pin) {
    volatile uint32_t* bop_data = reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[static_cast<size_t>(base)] + BOP_OFFSET);
    *bop_data = (1U << static_cast<size_t>(pin));
}

static inline void fast_clear_pin(GPIO_Base base, Pin_Number pin) {
    volatile uint32_t* bc_data = reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[static_cast<size_t>(base)] + BC_OFFSET);
    *bc_data = (1U << static_cast<size_t>(pin));
}

static inline bool fast_read_pin(GPIO_Base base, Pin_Number pin) {
    volatile uint32_t* istat_data = reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[static_cast<size_t>(base)] + ISTAT_OFFSET);
    return (*istat_data & (1U << static_cast<size_t>(pin))) != Clear;
}

static inline void fast_toggle_pin(GPIO_Base base, Pin_Number pin) {
    volatile uint32_t* octl_data = reinterpret_cast<volatile uint32_t*>(GPIO_baseAddress[static_cast<size_t>(base)] + OCTL_OFFSET);
    if (*octl_data & (1U << static_cast<size_t>(pin))) {
        fast_clear_pin(base, pin);
    } else {
        fast_set_pin(base, pin);
    }
}

} // namespace gpio
