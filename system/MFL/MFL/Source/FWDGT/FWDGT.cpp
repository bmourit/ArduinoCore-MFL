//
// MFL gd32f30x FWDGT peripheral register access in C++
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

#include "FWDGT.hpp"
#include "RCU.hpp"

namespace fwdgt {

FWDGT& FWDGT::get_instance() {
    static FWDGT instance;
    return instance;
}

FWDGT::FWDGT() {}

/**
 * @brief Enable watchdog
 *
 * Write magic value 0x0000CCCC to the Control Register (CTL) to enable the watchdog.
 *
 * @note This function is not protected against concurrent modification from interrupts.
 */
void FWDGT::enable() {
    write_register(*this, FWDGT_Regs::CTL, EnableValue);
}

/**
 * @brief Enables write access to the FWDGT registers.
 *
 * Writes the magic value 0x00005555 to the Control Register (CTL)
 * to allow modification of the FWDGT configuration registers.
 */
void FWDGT::write_enable() {
    write_register(*this, FWDGT_Regs::CTL, WriteEnable);
}

/**
 * @brief Disables write access to the FWDGT registers.
 *
 * Writes the magic value 0x00000000 to the Control Register (CTL)
 * to disable modification of the FWDGT configuration registers.
 */
void FWDGT::write_disable() {
    write_register(*this, FWDGT_Regs::CTL, Clear);
}

/**
 * @brief Enables or disables write access to the FWDGT registers.
 *
 * If `enable` is true, the magic value 0x00005555 is written to the
 * Control Register (CTL) to allow modification of the FWDGT configuration
 * registers. If `enable` is false, the magic value 0x00000000 is written to
 * the Control Register (CTL) to disable modification of the FWDGT configuration
 * registers.
 *
 * @param enable Set to true to enable write access, false to disable it.
 */
void FWDGT::set_write_enable(bool enable) {
    write_register(*this, FWDGT_Regs::CTL, enable ? WriteEnable : Clear);
}

/**
 * @brief Sets the prescaler value for the FWDGT.
 *
 * This function enables write access to the FWDGT registers and attempts to
 * set the prescaler to the specified value. It waits for the PUD flag to clear
 * before proceeding with the operation. If the PUD flag does not clear, the function
 * returns true indicating failure. Otherwise, it returns false indicating success.
 *
 * @param value The prescaler value to set.
 * @return false on success, true on failure.
 */
bool FWDGT::set_prescaler(Prescaler_Value value) {
    // Enable write access
    write_register(*this, FWDGT_Regs::CTL, WriteEnable);

    bool status = false;
    do {
        status = read_bit(*this, FWDGT_Regs::STAT, static_cast<uint32_t>(STAT_Bits::PUD));
    } while (status != false);

    // If the PUD flag didn't clear, return true for failure
    if (status != false) { return true; }

    // Set the prescaler value
    write_register(*this, FWDGT_Regs::PSC, static_cast<uint32_t>(value));

    return false; // Success
}

/**
 * @brief Returns the current prescaler value of the FWDGT.
 *
 * This function reads the FWDGT's prescaler value and returns it as an unsigned
 * 32-bit integer. The prescaler value is a 16-bit value, so the upper 16 bits
 * of the returned value are zero.
 *
 * @return The current prescaler value of the FWDGT as an unsigned 32-bit
 *         integer.
 */
uint32_t FWDGT::get_prescaler() {
    return read_bit_range(*this, FWDGT_Regs::PSC, static_cast<uint32_t>(PSC_Bits::PSC));
}

/**
 * @brief Sets the reload value of the FWDGT.
 *
 * This function enables write access to the FWDGT registers and attempts to
 * set the reload value to the specified value. It waits for the RUD flag to clear
 * before proceeding with the operation. If the RUD flag does not clear, the function
 * returns true indicating failure. Otherwise, it returns false indicating success.
 *
 * @param reload The reload value to set.
 * @return false on success, true on failure.
 */
bool FWDGT::set_reload(uint32_t reload) {
    // Enable write access
    write_register(*this, FWDGT_Regs::CTL, WriteEnable);

    bool status = false;
    do {
        status = read_bit(*this, FWDGT_Regs::STAT, static_cast<uint32_t>(STAT_Bits::RUD));
    } while (status != false);

    if (status != false) {
        return true;
    }
    write_register(*this, FWDGT_Regs::RLD, reload);

    return false;
}

/**
 * @brief Returns the current reload value of the FWDGT.
 *
 * This function reads the FWDGT's reload value and returns it as an unsigned
 * 32-bit integer. The reload value is a 16-bit value, so the upper 16 bits
 * of the returned value are zero.
 *
 * @return The current reload value of the FWDGT as an unsigned 32-bit
 *         integer.
 */
uint32_t FWDGT::get_reload() {
    return read_register<uint32_t>(*this, FWDGT_Regs::RLD);
}

/**
 * @brief Reloads the FWDGT counter.
 *
 * This function writes the magic value `ReloadValue` to the Control Register (CTL)
 * to reload the FWDGT counter, preventing a system reset. It is crucial to periodically
 * call this function to ensure the watchdog timer does not expire and reset the system.
 */
void FWDGT::reload_counter() {
    write_register(*this, FWDGT_Regs::CTL, ReloadValue);
}

/**
 * @brief Returns the status of a specified flag in the FWDGT.
 *
 * This function reads the FWDGT's status register (STAT) and returns the
 * current status of the specified flag. If the flag is not applicable to the
 * FWDGT, this function returns false.
 *
 * @param flag The status flag to retrieve. Must be a value from the
 *             Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool FWDGT::get_flag(Status_Flags flag) {
    return read_bit(*this, FWDGT_Regs::STAT, static_cast<uint32_t>(flag));
}


} // namespace fwdgt
