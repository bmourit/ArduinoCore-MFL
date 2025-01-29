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

#include <stdint.h>

#include "rtc_config.hpp"
#include "RegRW.hpp"

namespace rtc {

class RCU;
class PMU;
class BKP;

class RTC {
public:
    static RTC& get_instance();

    void set_prescaler(uint32_t prescaler);
    void set_alarm(uint32_t alarm);
    uint32_t get_divider();
    void start_configuration();
    void stop_configuration();
    void lwoff_wait();
    void sync_register_wait();
    uint32_t get_counter();
    void set_counter(uint32_t counter);
    bool get_flag(Status_Flags flag);
    void clear_flag(Status_Flags flag);
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    static inline constexpr uintptr_t RTC_baseAddress = 0x40002800U;

    inline volatile uint32_t* reg_address(RTC_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(RTC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    RTC();

    // Prevent copying or assigning
    RTC(const RTC&) = delete;
    RTC& operator=(const RTC&) = delete;

    mutable bool is_clock_enabled_;
};

} // namespace rtc
