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
 * @brief Resets the WWDGT peripheral by toggling its peripheral clock reset.
 *
 * This function enables the peripheral clock reset for the WWDGT by setting
 * the reset register, then disables the reset to complete the reset operation.
 * This effectively resets all registers of the WWDGT peripheral to their
 * default values.
 */
void WWDGT::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_WWDGTRST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_WWDGTRST, false);
}

/**
 * @brief Enables the WWDGT peripheral.
 *
 * This function enables the WWDGT peripheral by setting the WDGTEN bit in the
 * CTL register. Once enabled, the WWDGT starts counting down from the current
 * counter value and can generate an early warning interrupt.
 */
void WWDGT::enable() {
    write_bit(*this, WWDGT_Regs::CTL, static_cast<uint32_t>(CTL_Bits::WDGTEN), true);
}

/**
 * @brief Updates the WWDGT counter value.
 *
 * This function updates the current counter value of the WWDGT peripheral by
 * setting the CNT bits in the CTL register to the specified value. This
 * function can be used at any time, regardless of whether the WWDGT is enabled
 * or disabled. If the WWDGT is enabled, the counter value is updated immediately
 * and the WWDGT continues counting down from the new value.
 *
 * @param[in] value The new counter value to be set.
 */
void WWDGT::update_counter(uint16_t value) {
    write_bit_range(*this, WWDGT_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CNT), static_cast<uint32_t>(value));
}

/**
 * @brief Sets up the WWDGT peripheral with the specified counter value, window value, and prescaler.
 *
 * This function sets up the WWDGT peripheral by setting the CNT bits in the CTL
 * register to the specified counter value, the WIN bits in the CFG register to
 * the specified window value, and the PSC bits in the CFG register to the
 * specified prescaler value. The window value must be less than or equal to the
 * counter value, and the prescaler value must be one of the values listed in the
 * Prescaler_Values enumeration.
 *
 * @param[in] value The counter value to be set.
 * @param[in] window The window value to be set.
 * @param[in] prescaler The prescaler value to be set. Must be one of the
 *            values listed in the Prescaler_Values enumeration.
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
 * @brief Clears the WWDGT early warning interrupt flag.
 *
 * This function clears the WWDGT early warning interrupt flag (EWIF) by writing
 * a 1 to the EWIF bit in the STAT register. The EWIF is set when the WWDGT
 * counter value is equal to the window value set with the setup() function. The
 * EWIF is cleared when the WWDGT is reset or when this function is called.
 */
void WWDGT::clear_flag() {
    write_register(*this, WWDGT_Regs::STAT, Clear);
}

/**
 * @brief Enables or disables the WWDGT early warning interrupt.
 *
 * This function enables or disables the WWDGT early warning interrupt. If the
 * interrupt is enabled, the WWDGT generates an interrupt when the counter value
 * is equal to the window value set with the setup() function. If the interrupt
 * is disabled, the WWDGT does not generate an interrupt when the counter value
 * is equal to the window value set with the setup() function.
 *
 * @param[in] enable Set to true to enable the WWDGT early warning interrupt,
 *                   false to disable it.
 */
void WWDGT::set_interrupt_enable(bool enable) {
    write_bit(*this, WWDGT_Regs::CFG, static_cast<uint32_t>(CFG_Bits::EWIE), enable);
}


} // namespace wwdgt
