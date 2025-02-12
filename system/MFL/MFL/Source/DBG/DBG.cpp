//
// MFL gd32f30x DBG debug in C++
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

#include "DBG.hpp"
#include "RCU.hpp"

namespace armdbg {

ARMDBG& ARMDBG::get_instance() {
    static ARMDBG instance;
    return instance;
}

ARMDBG::ARMDBG() {}

/**
 * @brief Resets the DBG peripheral.
 *
 * This function resets the DBG peripheral by writing '0's to the DBG_CTL0 register
 * and thus clearing all bits in the register. This is the same as performing a
 * peripheral reset.
 */
void ARMDBG::reset() {
    write_register<uint32_t>(*this, DBG_Regs::CTL0, Clear);
}

/**
 * @brief Returns the debug ID of the DBG peripheral.
 *
 * This function reads the DBG_ID register and returns its value as a uint32_t.
 *
 * @return The debug ID of the DBG peripheral as a uint32_t.
 */
uint32_t ARMDBG::get_debug_id() {
    return read_register<uint32_t>(*this, DBG_Regs::ID);
}

/**
 * @brief Enables or disables debug with trace output.
 *
 * If the argument is true, this function enables debug with trace output. If the
 * argument is false, this function disables debug with trace output.
 *
 * @param enable If true, enables debug with trace output. If false, disables debug
 *               with trace output.
 */
void ARMDBG::set_debug_trace_enable(bool enable) {
    write_bit(*this, DBG_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::TRACE_IOEN), enable);
}

/**
 * @brief Enables or disables debug for the specified peripheral.
 *
 * If the argument is true, this function enables debug for the specified
 * peripheral. If the argument is false, this function disables debug for the
 * specified peripheral.
 *
 * @param peripheral The peripheral to enable or disable debug for.
 * @param enable If true, enables debug for the peripheral. If false, disables
 *               debug for the peripheral.
 */
void ARMDBG::set_peripheral_debug_enable(Debug_Peripheral peripheral, bool enable) {
    write_bit(*this, DBG_Regs::CTL0, static_cast<uint32_t>(peripheral), enable);
}

/**
 * @brief Enables or disables debugging during low power modes.
 *
 * This function sets the debug configuration for specific low power modes
 * by enabling or disabling it based on the `enable` parameter. It modifies
 * the corresponding bit in the DBG control register.
 *
 * @param type The type of low power mode to configure for debugging.
 * @param enable If true, enables debugging during the specified low power mode.
 *               If false, disables debugging during the specified low power mode.
 */
void ARMDBG::set_debug_low_power_enable(Low_Power_Debug type, bool enable) {
    write_bit(*this, DBG_Regs::CTL0, static_cast<uint32_t>(type), enable);
}


} // namespace armdbg
