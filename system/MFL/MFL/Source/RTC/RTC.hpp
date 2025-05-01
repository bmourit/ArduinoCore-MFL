//
// MFL gd32f30x RTC peripheral register access in C++
//
// Copyright (C) 2024 B. Mouritsen <bnmguy@gmail.com>. All rights reserved.
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

#include "rtc_config.hpp"
#include "RegRW.hpp"
#include "BKP.hpp"

namespace rtc {

class RTC {
public:
    static auto get_instance() -> RTC&;

    // Prescaler
    void set_prescaler(uint32_t prescaler);

    // Alarm
    void set_alarm(uint32_t alarm);

    // Divider
    auto get_divider() -> uint32_t;

    // Configuration
    void start_configuration();
    void stop_configuration();

    // Wait
    void lwoff_wait();
    void sync_register_wait();

    // Counter
    auto get_counter() -> uint32_t;
    void set_counter(uint32_t counter);

    // Interrupts and flags
    auto get_flag(Status_Flags flag) -> bool;
    void clear_flag(Status_Flags flag);
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    static inline constexpr uintptr_t RTC_baseAddress = 0x4000'2800U;

    [[nodiscard]] inline auto reg_address(RTC_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(RTC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    RTC();

    // Reference of the BPK instance
    bkp::BKP& bkp_;

    // Prevent copying or assigning
    RTC(const RTC&) = delete;
    auto operator=(const RTC&) -> RTC& = delete;

    mutable bool is_clock_enabled_;
};

} // namespace rtc
