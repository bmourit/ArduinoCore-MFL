//
// MFL gd32f30x WWDGT peripheral register access in C++
//
// Copyright (C) 2025 B. Mourit <bnmguy@gmail.com>
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

#include "WWDGT.hpp"
#include "RCU.hpp"

namespace wwdgt {

WWDGT& WWDGT::get_instance() {
    static WWDGT instance;
    return instance;
}

WWDGT::WWDGT() : is_clock_enabled_(false) {
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_WWDGT, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_WWDGTRST, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_WWDGTRST, false);
        is_clock_enabled_ = true;
    }
}

/**
 * Reset the WWDGT peripheral.
 *
 * This function triggers the WWDGT peripheral reset, which resets all WWDGT
 * registers to their default values.
 */
void WWDGT::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_WWDGTRST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_WWDGTRST, false);
}

/**
 * Enable the WWDGT peripheral.
 *
 * This function enables the WWDGT peripheral by setting the WDGTEN bit in the
 * Control Register (CTL). When the WWDGT is enabled, it starts counting down
 * from the initial value set with the update_counter() function.
 */
void WWDGT::enable() {
    write_bit(*this, WWDGT_Regs::CTL, static_cast<uint32_t>(CTL_Bits::WDGTEN), true);
}

/**
 * Updates the WWDGT counter value.
 *
 * This function updates the WWDGT counter value, which is the initial value from
 * which the WWDGT starts counting down when enabled. The counter value is a
 * 16-bit unsigned integer.
 *
 * @param[in] value The new counter value to be set.
 */
void WWDGT::update_counter(uint16_t value) {
    write_bit_range(*this, WWDGT_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CNT), static_cast<uint32_t>(value));
}

/**
 * Configures the WWDGT peripheral.
 *
 * This function configures the WWDGT peripheral by setting the counter value,
 * window value, and prescaler value. The counter value is the initial value
 * from which the WWDGT starts counting down when enabled. The window value
 * specifies the window for the early warning interrupt to be generated. The
 * prescaler value specifies the prescaler division factor for the WWDGT clock.
 *
 * @param[in] value The counter value to be set.
 * @param[in] window The window value to be set.
 * @param[in] prescaler The prescaler value to be set.
 */
void WWDGT::setup(uint16_t value, uint16_t window, Prescaler_Values prescaler) {
    write_bit_range(*this, WWDGT_Regs::CTL,
               static_cast<uint32_t>(CTL_Bits::CNT), static_cast<uint32_t>(value));
    write_bit_ranges(*this, WWDGT_Regs::CFG,
               static_cast<uint32_t>(CFG_Bits::WIN), static_cast<uint32_t>(window),
               static_cast<uint32_t>(CFG_Bits::PSC), static_cast<uint32_t>(prescaler));
}

/**
 * @brief Retrieves the status of the WWDGT early warning interrupt flag.
 *
 * This function returns the status of the WWDGT early warning interrupt flag
 * (EWIF). The EWIF is set when the WWDGT counter value is equal to the window
 * value set with the setup() function. The EWIF is cleared when the WWDGT is
 * reset or when the clear_flag() function is called.
 *
 * @return true if the EWIF is set, false otherwise.
 */
bool WWDGT::get_flag() {
    return read_bit(*this, WWDGT_Regs::STAT, static_cast<uint32_t>(STAT_Bits::EWIF));
}

/**
 * Clears the WWDGT early warning interrupt flag (EWIF).
 *
 * This function clears the EWIF by writing a 1 to the corresponding bit in
 * the STAT register. The EWIF is set when the WWDGT counter value is equal to
 * the window value set with the setup() function. The EWIF is cleared when the
 * WWDGT is reset or when this function is called.
 */
void WWDGT::clear_flag() {
    write_register(*this, WWDGT_Regs::STAT, Clear);
}

/**
 * Enables or disables the WWDGT early warning interrupt.
 *
 * This function enables or disables the WWDGT early warning interrupt by setting
 * or clearing the EWIE bit in the CFG register. When the EWIE is set, an interrupt
 * will be generated when the WWDGT counter value is equal to the window value set
 * with the setup() function.
 *
 * @param[in] enable If true, enables the early warning interrupt. If false,
 * disables the early warning interrupt.
 */
void WWDGT::set_interrupt_enable(bool enable) {
    write_bit(*this, WWDGT_Regs::CFG, static_cast<uint32_t>(CFG_Bits::EWIE), enable);
}


} // namespace wwdgt
