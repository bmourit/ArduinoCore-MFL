//
// MFL gd32f30x OB peripheral register access in C++
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

#include "fmc_config.hpp"
#include "RegRW.hpp"

namespace fmc {

class OB {
public:
    static auto get_instance() -> OB&;

    // Lock
    void ob_lock();
    void ob_unlock();

    // Erase
    FMC_Error_Type ob_erase();

    // Write protection
    FMC_Error_Type set_ob_write_protection(WP_Sector sector);
    auto get_ob_write_protection() -> uint32_t;

    // Security protection
    FMC_Error_Type set_ob_security_protection(OB_Security_Type type);
    auto get_ob_security_protection() -> bool;

    // User
    FMC_Error_Type set_ob_user(OB_Watchdog_Type type, OB_Deep_Sleep deepsleep, OB_Standby standby, OB_Boot_Bank bank);
    auto get_ob_user() -> uint8_t;

    // Data
    FMC_Error_Type set_ob_data(uint32_t address, uint8_t data);
    auto get_ob_data() -> uint16_t;

    // State
    FMC_Error_Type ob_get_bank0_state();
    FMC_Error_Type ob_get_bank1_state();

    // Wait state
    FMC_Error_Type ob_ready_wait_bank0(uint32_t timeout);
    FMC_Error_Type ob_ready_wait_bank1(uint32_t timeout);

    static inline constexpr uintptr_t FMC_baseAddress = 0x4002'2000U;
    static inline constexpr uintptr_t OB_baseAddress = 0x1FFF'F800U;

    [[nodiscard]] inline auto reg_address(OB_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(OB_baseAddress + static_cast<uint32_t>(reg));
    }

    [[nodiscard]] inline auto reg_address(FMC_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(FMC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    OB();

    // Prevent copying or assigning
    OB(const OB&) = delete;
    auto operator=(const OB&) -> OB& = delete;
};

} // namespace fmc
