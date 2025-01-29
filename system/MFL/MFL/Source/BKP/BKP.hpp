//
// MFL gd32f30x BKP peripheral register access in C++
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

#include "bkp_config.hpp"
#include "RegRW.hpp"

namespace bkp {

class RCU;

class BKP {
public:
    static BKP& get_instance();

    void reset();
    void set_data(Backup_Data data_offset, uint16_t data);
    uint16_t get_data(Backup_Data data_offset);

    void set_rtc_output_calibration_enable(bool enable);
    void set_rtc_output_signal_enable(bool enable);
    void set_rtc_output_pulse(Output_Pulse pulse);
    void set_rtc_clock_divider(Clock_Divider divider);
    void set_rtc_clock_calibration_type(Calibration_Type type);
    void set_rtc_calibration_value(uint8_t value);

    void set_tamper_detection_enable(bool enable);
    void set_tamper_level(Tamper_Level level);
    void set_tamper_interrupt_enable(bool enable);
    void tamper_interrupt_enable();
    void tamper_interrupt_disable();

    bool get_flag(Status_Flags flag);
    void clear_flag(Clear_Flags flag);
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Clear_Flags flag);

    static inline constexpr uintptr_t BKP_baseAddress = 0x40006C00U;

    inline volatile uint32_t* reg_address(BKP_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(BKP_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    BKP();

    mutable bool is_clock_enabled_;
};


} // namespace bkp
