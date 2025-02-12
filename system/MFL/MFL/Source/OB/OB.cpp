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

#include "OB.hpp"
#include "RCU.hpp"

namespace fmc {

OB& OB::get_instance() {
    static OB instance;
    return instance;
}

OB::OB() {}

/**
 * @brief Set the Option Bytes Lock (OBWEN) bit in the FMC Control
 *        Register (CTL0). This will lock the Option Bytes, preventing
 *        any accidental writes or erases.
 */
void OB::ob_lock() {
    write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBWEN), true);
}

/**
 * @brief Unlock the Option Bytes.
 *
 * If the Option Bytes Write Enable (OBWEN) bit in the FMC Control
 * Register (CTL0) is set, this function writes the unlock keys to the
 * FMC Option Bytes Key Register (OBKEY). This will unlock the Option
 * Bytes, allowing writes or erases.
 */
void OB::ob_unlock() {
    if (read_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBWEN))) {
        write_register(*this, FMC_Regs::OBKEY, static_cast<uint32_t>(Unlock_Key0));
        write_register(*this, FMC_Regs::OBKEY, static_cast<uint32_t>(Unlock_Key1));
    }
}

/**
 * @brief Erase the Option Bytes.
 *
 * This function erases the Option Bytes if the Option Bytes Erase
 * (OBER) bit in the FMC Control Register (CTL0) is set to 1 and
 * the Option Bytes Write Enable (OBWEN) bit is set to 0.
 *
 * @return The resulting state of the FMC after the erase operation
 *         has completed. Will be one of the FMC_Error_Type enumeration
 *         values.
 */
FMC_Error_Type OB::ob_erase() {
    uint16_t value = 0xA5U;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);
    FMC_Error_Type state = ob_ready_wait_bank0(timeout);

    if (get_ob_security_protection() == true) {
        value = 0xBBU;
    }

    if (state == FMC_Error_Type::READY) {
        write_bits_sequence(*this, FMC_Regs::CTL0,
                            static_cast<uint32_t>(CTL0_Bits::OBER), true,
                            static_cast<uint32_t>(CTL0_Bits::START), true);
        // Wait until ready
        state = ob_ready_wait_bank0(timeout);
        if (state == FMC_Error_Type::READY) {
            write_bits_sequence(*this, FMC_Regs::CTL0,
                                static_cast<uint32_t>(CTL0_Bits::OBER), false,
                                static_cast<uint32_t>(CTL0_Bits::OBPG), true);
            write_bit_range(*this, OB_Regs::SPC, static_cast<uint32_t>(SPC_Bits::SPC), value);
            // Wait until ready
            state = ob_ready_wait_bank0(timeout);
            if (state != FMC_Error_Type::TIMEOUT) {
                write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), false);
            }
        } else {
            if (state != FMC_Error_Type::TIMEOUT) {
                write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), false);
            }
        }
    }

    return state;
}

/**
 * @brief Configure write protection for the specified sector.
 *
 * This function sets the write protection for a specific sector by
 * modifying the Option Bytes write protection registers. It waits
 * for the FMC to be ready before and after changing the protection
 * settings. The function operates on multiple bytes of the write
 * protection registers depending on the sector specified.
 *
 * @param sector The sector for which write protection is to be configured.
 *
 * @return The resulting state of the FMC after the write protection
 *         operation. Will be one of the FMC_Error_Type enumeration values.
 */
FMC_Error_Type OB::set_ob_write_protection(WP_Sector sector) {
    uint16_t wp_sector_value = 0U;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);
    FMC_Error_Type state = ob_ready_wait_bank0(timeout);
    uint32_t wp_sector = ~(1U << static_cast<uint32_t>(sector));

    if (state == FMC_Error_Type::READY) {
        write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), true);
        wp_sector_value = (wp_sector & 0x000000FFU);
        if (wp_sector_value != 0xFFU) {
            write_register(*this, OB_Regs::WP0, wp_sector_value);
            // Wait until ready
            state = ob_ready_wait_bank0(timeout);
        }
        wp_sector_value = ((wp_sector & 0x0000FF00U) >> 8U);
        if ((state == FMC_Error_Type::READY) && (wp_sector_value != 0xFFU)) {
            write_register(*this, OB_Regs::WP1, wp_sector_value);
            // Wait until ready
            state = ob_ready_wait_bank0(timeout);
        }
        wp_sector_value = ((wp_sector & 0x00FF0000U) >> 16U);
        if ((state == FMC_Error_Type::READY) && (wp_sector_value != 0xFFU)) {
            write_register(*this, OB_Regs::WP2, wp_sector_value);
            // Wait until ready
            state = ob_ready_wait_bank0(timeout);
        }
        wp_sector_value = ((wp_sector & 0xFF000000U) >> 24U);
        if ((state == FMC_Error_Type::READY) && (wp_sector_value != 0xFFU)) {
            write_register(*this, OB_Regs::WP3, wp_sector_value);
            // Wait until ready
            state = ob_ready_wait_bank0(timeout);
        }
        if (state != FMC_Error_Type::TIMEOUT) {
            write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), false);
        }
    }

    return state;
}

/**
 * @brief Read the current write protection settings.
 *
 * This function reads the value of the Option Bytes write protection
 * registers. The value returned is a 32-bit word with the lower 8 bits
 * representing the write protection configuration for sectors 0-7,
 * the next 8 bits representing sectors 8-15, and so on.
 *
 * @return The current write protection settings
 */
uint32_t OB::get_ob_write_protection() {
    return read_register<uint32_t>(*this, FMC_Regs::WP);
}

/**
 * @brief Set the security protection level for the Option Bytes.
 *
 * This function configures the security protection by modifying the
 * Option Bytes. It waits for the FMC to be ready before initiating
 * the protection configuration and verifies readiness afterward.
 *
 * The function writes to the Option Bytes Protection (SPC) register
 * based on the specified security type. If the operation is successful,
 * it disables the Option Bytes Programming (OBPG) bit.
 *
 * @param type The security protection type to be set, specified as
 *             an OB_Security_Type enumeration value.
 *
 * @return The resulting state of the FMC after attempting to set the
 *         security protection. The return value will be one of the
 *         FMC_Error_Type enumeration values indicating the success or
 *         failure of the operation.
 */
FMC_Error_Type OB::set_ob_security_protection(OB_Security_Type type) {
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);
    FMC_Error_Type state = ob_ready_wait_bank0(timeout);

    if (state == FMC_Error_Type::READY) {
        write_bits_sequence(*this, FMC_Regs::CTL0,
                            static_cast<uint32_t>(CTL0_Bits::OBER), true,
                            static_cast<uint32_t>(CTL0_Bits::START), true);
        // Wait until ready
        state = ob_ready_wait_bank0(timeout);
        if (state == FMC_Error_Type::READY) {
            write_bits_sequence(*this, FMC_Regs::CTL0,
                                static_cast<uint32_t>(CTL0_Bits::OBER), false,
                                static_cast<uint32_t>(CTL0_Bits::OBPG), true);
            write_bit_range(*this, OB_Regs::SPC, static_cast<uint32_t>(SPC_Bits::SPC), static_cast<uint32_t>(type));
            // Wait until ready
            state = ob_ready_wait_bank0(timeout);
            if (state != FMC_Error_Type::TIMEOUT) {
                write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), false);
            }
        } else {
            if (state != FMC_Error_Type::TIMEOUT) {
                write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBER), false);
            }
        }
    }

    return state;
}

/**
 * @brief Check the current security protection status.
 *
 * This function reads the security protection bit from the Option Bytes
 * status register to determine if security protection is enabled.
 *
 * @return True if security protection is enabled, false otherwise.
 */
bool OB::get_ob_security_protection() {
    return read_bit(*this, FMC_Regs::OBSTAT, static_cast<uint32_t>(OBSTAT_Bits::SPC));
}

/**
 * @brief Set the user configuration for the Option Bytes.
 *
 * This function sets the user configuration values by modifying the
 * Option Bytes. It waits for the FMC to be ready before initiating
 * the user configuration and verifies readiness afterward.
 *
 * The function writes to the Option Bytes User (USER) register based
 * on the specified parameters. If the operation is successful, it
 * disables the Option Bytes Programming (OBPG) bit.
 *
 * @param type The watchdog type to be set, specified as an
 *             OB_Watchdog_Type enumeration value.
 * @param deepsleep The deep sleep option to be set, specified as an
 *                  OB_Deep_Sleep enumeration value.
 * @param standby The standby option to be set, specified as an
 *                OB_Standby enumeration value.
 * @param bank The boot bank option to be set, specified as an
 *             OB_Boot_Bank enumeration value.
 *
 * @return The resulting state of the FMC after attempting to set the
 *         user configuration. The return value will be one of the
 *         FMC_Error_Type enumeration values indicating the success or
 *         failure of the operation.
 */
FMC_Error_Type OB::set_ob_user(OB_Watchdog_Type type, OB_Deep_Sleep deepsleep, OB_Standby standby, OB_Boot_Bank bank) {
    FMC_Error_Type state = FMC_Error_Type::READY;
    uint32_t user_op;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);
    state = ob_ready_wait_bank0(timeout);
    if (state == FMC_Error_Type::READY) {
        write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), true);
        user_op = static_cast<uint32_t>(bank) | static_cast<uint32_t>(type) | static_cast<uint32_t>(deepsleep) | static_cast<uint32_t>(standby) | 0x000000F0U;
        write_bit_range(*this, OB_Regs::USER, static_cast<uint32_t>(USER_Bits::USER), user_op);
        // Wait until ready
        state = ob_ready_wait_bank0(timeout);
        if (state != FMC_Error_Type::TIMEOUT) {
            write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), false);
        }
    }

    return state;
}

/**
 * @brief Retrieve the current user configuration settings.
 *
 * This function reads and returns the user configuration settings from
 * the Option Bytes status register. The value returned is derived by
 * shifting the relevant portion of the register contents and casting
 * it to a uint8_t.
 *
 * @return The current user configuration settings as an 8-bit value.
 */
uint8_t OB::get_ob_user() {
    uint32_t value = read_register<uint32_t>(*this, FMC_Regs::OBSTAT);
    value = (value >> 2U);
    return static_cast<uint8_t>(value);
}

/**
 * @brief Set the value of the specified Option Bytes data.
 *
 * This function sets the value of the specified Option Bytes data
 * by writing the given data to the specified address. It waits for
 * the FMC to be ready before and after writing the data. The
 * function returns the resulting state of the FMC after the write
 * operation has completed. The return value will be one of the
 * FMC_Error_Type enumeration values indicating the success or
 * failure of the operation.
 *
 * @param address The address of the Option Bytes data to be set.
 * @param data The value to be written to the specified address.
 *
 * @return The resulting state of the FMC after the write operation.
 */
FMC_Error_Type OB::set_ob_data(uint32_t address, uint8_t data) {
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);
    FMC_Error_Type state = ob_ready_wait_bank0(timeout);

    if (state == FMC_Error_Type::READY) {
        write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), true);
        (*(volatile uint16_t *)(uint32_t)(address)) = data;
        // Wait until ready
        state = ob_ready_wait_bank0(timeout);
        if (state != FMC_Error_Type::TIMEOUT) {
            write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::OBPG), false);
        }
    }

    return state;
}

/**
 * @brief Retrieve the current Option Bytes data.
 *
 * This function reads and returns the value of the Option Bytes data
 * register. The value returned is derived by shifting the relevant
 * portion of the register contents and casting it to a uint16_t.
 *
 * @return The current Option Bytes data as a 16-bit value.
 */
uint16_t OB::get_ob_data() {
    uint32_t value = read_register<uint32_t>(*this, FMC_Regs::OBSTAT);
    value = (value >> 10U);
    return static_cast<uint16_t>(value);
}

/**
 * @brief Get the current state of the Option Bytes bank 0.
 *
 * This function checks the current state of the Option Bytes bank 0
 * by reading the STAT0 register. The state is then returned as a
 * FMC_Error_Type enumeration value.
 *
 * @return The current state of the Option Bytes bank 0 as a
 *         FMC_Error_Type enumeration value.
 */
FMC_Error_Type OB::ob_get_bank0_state() {
    FMC_Error_Type state = FMC_Error_Type::READY;

    if (read_bit(*this, FMC_Regs::STAT0, static_cast<uint32_t>(STAT0_Bits::BUSY))) {
        state = FMC_Error_Type::BUSY;
    } else if (read_bit(*this, FMC_Regs::STAT0, static_cast<uint32_t>(STAT0_Bits::WPERR))) {
        state = FMC_Error_Type::WP_ERROR;
    } else if (read_bit(*this, FMC_Regs::STAT0, static_cast<uint32_t>(STAT0_Bits::PGERR))) {
        state = FMC_Error_Type::PG_ERROR;
    }

    return state;
}

/**
 * @brief Get the current state of the Option Bytes bank 1.
 *
 * This function checks the current state of the Option Bytes bank 1
 * by reading the STAT1 register. The state is then returned as a
 * FMC_Error_Type enumeration value.
 *
 * @return The current state of the Option Bytes bank 1 as a
 *         FMC_Error_Type enumeration value.
 */
FMC_Error_Type OB::ob_get_bank1_state() {
    FMC_Error_Type state = FMC_Error_Type::READY;

    if (read_bit(*this, FMC_Regs::STAT1, static_cast<uint32_t>(STAT1_Bits::BUSY))) {
        state = FMC_Error_Type::BUSY;
    } else if (read_bit(*this, FMC_Regs::STAT1, static_cast<uint32_t>(STAT1_Bits::WPERR))) {
        state = FMC_Error_Type::WP_ERROR;
    } else if (read_bit(*this, FMC_Regs::STAT1, static_cast<uint32_t>(STAT1_Bits::PGERR))) {
        state = FMC_Error_Type::PG_ERROR;
    }

    return state;
}

/**
 * @brief Waits until the Option Bytes bank 0 is ready.
 *
 * This function waits until the Option Bytes bank 0 is ready by
 * polling the STAT0 register. The function will timeout if the
 * specified timeout value is reached. The return value will be a
 * FMC_Error_Type enumeration value indicating the success or
 * failure of the operation.
 *
 * @param timeout The timeout value in number of iterations.
 *
 * @return The state of the Option Bytes bank 0 as a
 *         FMC_Error_Type enumeration value.
 */
FMC_Error_Type OB::ob_ready_wait_bank0(uint32_t timeout) {
    FMC_Error_Type state = FMC_Error_Type::BUSY;

    // Wait until ready
    do {
        state = ob_get_bank0_state();
        timeout = timeout - 1;
    } while ((state == FMC_Error_Type::BUSY) && (timeout != Clear));

    if (state == FMC_Error_Type::BUSY) {
        state = FMC_Error_Type::TIMEOUT;
    }

    return state;
}

/**
 * @brief Waits until the Option Bytes bank 1 is ready.
 *
 * This function waits until the Option Bytes bank 1 is ready by
 * polling the STAT1 register. The function will timeout if the
 * specified timeout value is reached. The return value will be a
 * FMC_Error_Type enumeration value indicating the success or
 * failure of the operation.
 *
 * @param timeout The timeout value in number of iterations.
 *
 * @return The state of the Option Bytes bank 1 as a
 *         FMC_Error_Type enumeration value.
 */
FMC_Error_Type OB::ob_ready_wait_bank1(uint32_t timeout) {
    FMC_Error_Type state = FMC_Error_Type::BUSY;

    // Wait until ready
    do {
        state = ob_get_bank1_state();
        timeout = timeout - 1;
    } while ((state == FMC_Error_Type::BUSY) && (timeout != Clear));

    if (state == FMC_Error_Type::BUSY) {
        state = FMC_Error_Type::TIMEOUT;
    }

    return state;
}

} // namespace fmc
