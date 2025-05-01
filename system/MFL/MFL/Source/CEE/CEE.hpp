//
// MFL gd32f30x CEE register access in C++
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

#include "cee_config.hpp"
#include "RegRW.hpp"

namespace cee {

class CEE {
public:
    static auto get_instance() -> CEE&;

    // Enable
    void set_enhanced_mode_enable(bool enable);

    static inline constexpr uintptr_t CEE_baseAddress = 0x4002'103CU;

    // Register access needed for read_bit/write_bit functionality
    [[nodiscard]] inline auto reg_address(CEE_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(CEE_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    CEE();

    // Prevent copying or assigning
    CEE(const CEE&) = delete;
    auto operator=(const CEE&) -> CEE& = delete;
};

} // namespace exti
