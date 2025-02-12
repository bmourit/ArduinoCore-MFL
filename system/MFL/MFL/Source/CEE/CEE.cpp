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

#include "CEE.hpp"

namespace cee {

CEE& CEE::get_instance() {
    static CEE instance;
    return instance;
}

CEE::CEE() {}

/**
 * @brief Enables or disables the code execution enhancement bit.
 *
 * This function sets or clears the CEE_EN bit in the CEE register
 * to enable or disable the code execution enhancement mode.
 *
 * @param enable Set to true to enable the code execution enhancement, false to disable it.
 */
void CEE::set_enhanced_mode_enable(bool enable) {
    write_bit(*this, CEE_Regs::CEE, static_cast<uint32_t>(CEE_Bits::CEE_EN), enable);
}

} // namespace cee
