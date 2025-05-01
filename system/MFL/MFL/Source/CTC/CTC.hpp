//
// MFL gd32f30x CTC peripheral register access in C++
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

#include "ctc_config.hpp"
#include "RegRW.hpp"

namespace ctc {

class CTC {
public:
    static auto get_instance() -> CTC&;

    // Reset
    void reset();

    // Counter
    void set_trim_counter_enable(bool enable);

    // Trim
    void set_irc48m_trim(uint8_t value);
    void set_hardware_auto_trim_enable(bool enable);
    void set_clock_trim_limit(uint8_t limit);
    void set_trim_counter_reload(uint16_t reload);
    auto get_trim_counter_capture() -> uint16_t;
    auto get_trim_counter_direction() -> bool;
    auto get_trim_counter_reload() -> uint16_t;
    auto get_trim_irc48m() -> uint8_t;

    // Reference source
    void generate_reference_source_pulse();
    void set_reference_source_polarity(Reference_Polarity polarity);
    void set_refenece_source_signal(Reference_Select reference);
    void set_reference_source_prescaler(Reference_Prescaler prescaler);

    // Interrupts and flags
    auto get_flag(Status_Flags flag) -> bool;
    void clear_flag(Clear_Flags flag);
    auto get_interrupt_flag(Interrupt_Flags flag) -> bool;
    void clear_interrupt_flag(Clear_Flags flag);
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    static inline constexpr uintptr_t CTC_baseAddress = 0x4000'C800U;

    [[nodiscard]] inline auto reg_address(CTC_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(CTC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    CTC();

    // Prevent copying or assigning
    CTC(const CTC&) = delete;
    auto operator=(const CTC&) -> CTC& = delete;

    mutable bool is_clock_enabled_;
};

} // namespace ctc
