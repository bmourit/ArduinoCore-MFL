//
// MFL gd32f30x FWDGT peripheral register access in C++
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

#include "fwdgt_config.hpp"
#include "RegRW.hpp"

namespace fwdgt {

class FWDGT {
public:
    static auto get_instance() -> FWDGT&;

    // Enable
    void enable();

    // Write
    void write_enable();
    void write_disable();
    void set_write_enable(bool enable);

    // Prescaler
    auto set_prescaler(Prescaler_Value value) -> bool;
    auto get_prescaler() -> uint32_t;
    auto set_reload_prescaler(uint32_t reload, Prescaler_Value value) -> bool;

    // Reload
    auto set_reload(uint32_t reload) -> bool;
    auto get_reload() -> uint32_t;
    void reload_counter();

    // Flags
    auto get_flag(Status_Flags flag) -> bool;

    static inline constexpr uintptr_t FWDGT_baseAddress = 0x4000'3000U;

    [[nodiscard]] inline auto reg_address(FWDGT_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(FWDGT_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    FWDGT();

    // Prevent copying or assigning
    FWDGT(const FWDGT&) = delete;
    auto operator=(const FWDGT&) -> FWDGT& = delete;
};

} // namespace fwdgt
