//
// MFL gd32f30x PMU peripheral register access in C++
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

#include "pmu_config.hpp"
#include "RegRW.hpp"

namespace pmu {

class RCU;

class PMU {
public:
    static PMU& get_instance();

    // Reset
    void reset();
    // Low voltage detection
    void lvd_enable(LVD_Threshold threshold);
    void lvd_disable();
    // Output voltage
    void set_ldo_output(Output_Voltage level);
    // High driver mode
    void set_high_driver_enable(bool enable);
    void high_driver_switch(bool enable);
    // Low driver mode
    void set_low_driver_on_deep_sleep_enable(bool enable);
    void set_low_driver_on_deep_sleep(Low_Driver mode);
    // Driver selection for power modes
    void set_driver_on_low_power(Power_Driver driver);
    void set_driver_on_normal_power(Power_Driver driver);
    // Sleep/Standby
    void set_standby_enable();
    void set_sleep_enable(PMU_Commands cmd);
    void set_deep_sleep_enable(Power_Driver driver, PMU_Commands command, bool enable);
    void set_sleep_mode_command(uint8_t value);
    void set_deep_sleep_mode_command(uint8_t value);
    void set_standby_mode(void);
    // Wakeup
    void set_wakeup_pin_enable(bool enable);
    // Backup
    void set_backup_write_enable(bool enable);

    // Flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Clear_Flags flag);

    static inline constexpr uintptr_t PMU_baseAddress = 0x40007000U;

    inline volatile uint32_t* reg_address(PMU_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(PMU_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    PMU();

    // Prevent copying or assigning
    PMU(const PMU&) = delete;
    PMU& operator=(const PMU&) = delete;

    mutable bool is_clock_enabled_;
};


} // namespace pmu

extern pmu::PMU& PMU_I;
