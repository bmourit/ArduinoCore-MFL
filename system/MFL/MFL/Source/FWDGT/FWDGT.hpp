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

#include <stdint.h>

#include "fwdgt_config.hpp"
#include "RegRW.hpp"

namespace fwdgt {

class RCU;

class FWDGT {
public:
    static FWDGT& get_instance();

    void enable();
    void write_enable();
    void write_disable();
    void set_write_enable(bool enable);
    bool set_prescaler(Prescaler_Value value);
    uint32_t get_prescaler();
    bool set_reload_prescaler(uint32_t reload, Prescaler_Value value);
    bool set_reload(uint32_t reload);
    uint32_t get_reload();
    void reload_counter();
    bool get_flag(Status_Flags flag);

    static inline constexpr uintptr_t FWDGT_baseAddress = 0x40003000U;

    inline volatile uint32_t* reg_address(FWDGT_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(FWDGT_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    FWDGT();

    // Prevent copying or assigning
    FWDGT(const FWDGT&) = delete;
    FWDGT& operator=(const FWDGT&) = delete;
};

} // namespace fwdgt
