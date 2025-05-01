//
// MFL gd32f30x FMC peripheral register access in C++
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

#include "FMC.hpp"
#include "RCU.hpp"

namespace fmc {

auto FMC::get_instance() -> FMC& {
    static FMC instance;
    return instance;
}

FMC::FMC() = default;

/**
 * @brief Unlock the FMC memory regions.
 *
 * This function unlocks the FMC memory regions by writing the unlock keys to the
 * FMC_KEY0 and FMC_KEY1 registers. This function checks if the memory regions are
 * already unlocked and only writes the keys if they are not.
 */
void FMC::unlock() {
    if (read_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::LK))) {
        write_register(*this, FMC_Regs::KEY0, Unlock_Key0);
        write_register(*this, FMC_Regs::KEY0, Unlock_Key1);
    }

    if (get_fmc_size() > Bank0_Size) {
        if (read_bit(*this, FMC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::LK))) {
            write_register(*this, FMC_Regs::KEY1, Unlock_Key0);
            write_register(*this, FMC_Regs::KEY1, Unlock_Key1);
        }
    }
}

/**
 * @brief Unlock the FMC bank0 memory region.
 *
 * This function unlocks the FMC memory region of bank0 by writing the unlock keys to the
 * FMC_KEY0 register. This function checks if the memory region is already unlocked and only
 * writes the keys if it is not.
 */
void FMC::unlock_bank0() {
    if (read_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::LK))) {
        write_register(*this, FMC_Regs::KEY0, Unlock_Key0);
        write_register(*this, FMC_Regs::KEY0, Unlock_Key1);
    }
}

/**
 * @brief Unlock the FMC bank1 memory region.
 *
 * This function unlocks the FMC memory region of bank1 by writing the unlock keys to the
 * FMC_KEY1 register. This function checks if the memory region is already unlocked and only
 * writes the keys if it is not.
 */
void FMC::unlock_bank1() {
    if (read_bit(*this, FMC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::LK))) {
        write_register(*this, FMC_Regs::KEY1, Unlock_Key0);
        write_register(*this, FMC_Regs::KEY1, Unlock_Key1);
    }
}

/**
 * @brief Lock the FMC bank0 and bank1 memory regions.
 *
 * This function locks the FMC memory regions of bank0 and bank1 by setting the LK bit in
 * the FMC_CTL0 and FMC_CTL1 registers. This prevents any further write operations to the
 * FMC memory regions.
 */
void FMC::lock() {
    write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::LK), true);

    if (get_fmc_size() > Bank0_Size) {
        write_bit(*this, FMC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::LK), true);
    }
}

/**
 * @brief Lock the FMC bank0 memory region.
 *
 * This function locks the FMC memory region of bank0 by setting the LK bit in
 * the FMC_CTL0 register. This prevents any further write operations to the
 * FMC bank0 memory region.
 */
void FMC::lock_bank0() {
    write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::LK), true);
}

/**
 * @brief Lock the FMC bank1 memory region.
 *
 * This function locks the FMC memory region of bank1 by setting the LK bit in
 * the FMC_CTL1 register. This prevents any further write operations to the
 * FMC bank1 memory region.
 */
void FMC::lock_bank1() {
    write_bit(*this, FMC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::LK), true);
}

/**
 * @brief Perform a mass erase on the FMC.
 *
 * This function performs a mass erase on the FMC by setting the MER bit and the
 * START bit in the FMC_CTL0 and FMC_CTL1 registers. The function waits until the
 * erase operation is complete and then clears the MER bit.
 *
 * If the FMC has a size greater than the size of bank0, the function will also
 * perform a mass erase on bank1.
 *
 * @return The state of the FMC after the mass erase operation.
 */
auto FMC::mass_erase() -> FMC_Error_Type {
    FMC_Error_Type state = ready_wait_bank0(Timeout_Count);

    if (state == FMC_Error_Type::READY) {
        write_bits_sequence(*this, FMC_Regs::CTL0,
                            static_cast<uint32_t>(CTL0_Bits::MER), true,
                            static_cast<uint32_t>(CTL0_Bits::START), true);
        // Wait until ready
        state = ready_wait_bank0(Timeout_Count);
        write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::MER), false);
    }

    if (get_fmc_size() > Bank0_Size) {
        state = ready_wait_bank1(Timeout_Count);
        if (state == FMC_Error_Type::READY) {
            write_bits_sequence(*this, FMC_Regs::CTL1,
                                static_cast<uint32_t>(CTL1_Bits::MER), true,
                                static_cast<uint32_t>(CTL1_Bits::START), true);
            // Wait until ready
            state = ready_wait_bank1(Timeout_Count);
            write_bit(*this, FMC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::MER), false);
        }
    }

    return state;
}

/**
 * @brief Erase a page in the FMC.
 *
 * This function erases a page in the FMC by setting the PER bit and the
 * START bit in the FMC_CTL0 or FMC_CTL1 registers, depending on the address
 * and size of the FMC. The function waits until the erase operation is complete
 * and then returns the state of the FMC.
 *
 * @param[in] address The address of the page to erase.
 *
 * @return The state of the FMC after the erase operation.
 */
auto FMC::erase_page(uint32_t address) -> FMC_Error_Type {
    // Choose bank based on FMC size or address threshold
    bool is_bank0 = (get_fmc_size() <= Bank0_Size || address < Bank0_End_Address);

    // Pick the correct CTL register
    auto ctl_reg = is_bank0 ? FMC_Regs::CTL0 : FMC_Regs::CTL1;
    // Unify PER bit to a uint32_t to avoid mixing enum classes
    uint32_t per_bit = is_bank0
        ? static_cast<uint32_t>(CTL0_Bits::PER)
        : static_cast<uint32_t>(CTL1_Bits::PER);
    // Unify START bit to a uint32_t to avoid mixing enum classes
    uint32_t start_bit = is_bank0
        ? static_cast<uint32_t>(CTL0_Bits::START)
        : static_cast<uint32_t>(CTL1_Bits::START);
    // Pick the correct ADDR register
    auto addr_reg = is_bank0 ? FMC_Regs::ADDR0 : FMC_Regs::ADDR1;

    return erase_word_bank(address, Timeout_Count, ctl_reg, per_bit, start_bit, addr_reg);
}

/**
 * @brief Erase the entire Bank 0 in the FMC.
 *
 * This function erases the entire Bank 0 in the FMC by setting the MER bit and
 * the START bit in the FMC_CTL0 register. The function waits until the erase
 * operation is complete and then returns the state of the FMC.
 *
 * @return The state of the FMC after the erase operation.
 */
auto FMC::erase_bank0() -> FMC_Error_Type {
    FMC_Error_Type state = ready_wait_bank0(Timeout_Count);
    if (state != FMC_Error_Type::READY) {
        return state;
    }

    write_bits_sequence(*this, FMC_Regs::CTL0,
                        static_cast<uint32_t>(CTL0_Bits::MER), true,
                        static_cast<uint32_t>(CTL0_Bits::START), true);

    // Wait until ready
    state = ready_wait_bank0(Timeout_Count);
    write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::MER), false);

    return state;
}

/**
 * @brief Erase the entire Bank 1 in the FMC.
 *
 * This function erases the entire Bank 1 in the FMC by setting the MER bit and
 * the START bit in the FMC_CTL1 register. The function waits until the erase
 * operation is complete and then returns the state of the FMC.
 *
 * @return The state of the FMC after the erase operation.
 */
auto FMC::erase_bank1() -> FMC_Error_Type {
    FMC_Error_Type state = ready_wait_bank1(Timeout_Count);
    if (state != FMC_Error_Type::READY) {
        return state;
    }

    write_bits_sequence(*this, FMC_Regs::CTL1,
                        static_cast<uint32_t>(CTL1_Bits::MER), true,
                        static_cast<uint32_t>(CTL1_Bits::START), true);

    // Wait until ready
    state = ready_wait_bank1(Timeout_Count);
    write_bit(*this, FMC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::MER), false);

    return state;
}

/**
 * @brief Programs a word to the FMC.
 *
 * This function programs a word to the FMC at the specified address. The
 * function determines which bank to use based on the address and the size of the
 * FMC. If the address is within the range of Bank 0, the function will use Bank 0
 * otherwise it will use Bank 1. The function waits until the program operation is
 * complete and then returns the state of the FMC.
 *
 * @param address The address to program the word to.
 * @param data The data to be programmed.
 * @return The state of the FMC after the program operation.
 */
auto FMC::program_word(uint32_t address, uint32_t data) -> FMC_Error_Type {
    // Choose bank based on FMC size or address threshold
    bool is_bank0 = (get_fmc_size() <= Bank0_Size || address < Bank0_End_Address);

    // Pick the correct CTL register and PG bit
    auto ctl_reg = is_bank0 ? FMC_Regs::CTL0 : FMC_Regs::CTL1;
    // Unify PG bit to a uint32_t to avoid mixing enum classes
    uint32_t pg_bit = is_bank0
        ? static_cast<uint32_t>(CTL0_Bits::PG)
        : static_cast<uint32_t>(CTL1_Bits::PG);

    return program_word_to_bank(address, data, Timeout_Count, ctl_reg, pg_bit);
}

/**
 * @brief Programs a halfword to the FMC.
 *
 * This function programs a halfword to the FMC at the specified address. The
 * function determines which bank to use based on the address and the size of the
 * FMC. If the address is within the range of Bank 0, the function will use Bank 0
 * otherwise it will use Bank 1. The function waits until the program operation is
 * complete and then returns the state of the FMC.
 *
 * @param address The address to program the halfword to.
 * @param data The data to be programmed.
 * @return The state of the FMC after the program operation.
 */
auto FMC::program_halfword(uint32_t address, uint16_t data) -> FMC_Error_Type {
    // Choose bank based on FMC size or address threshold
    bool is_bank0 = (get_fmc_size() <= Bank0_Size || address < Bank0_End_Address);

    // Pick the correct CTL register and PG bit
    auto ctl_reg = is_bank0 ? FMC_Regs::CTL0 : FMC_Regs::CTL1;
    // Unify PG bit to a uint32_t to avoid mixing enum classes
    uint32_t pg_bit = is_bank0
        ? static_cast<uint32_t>(CTL0_Bits::PG)
        : static_cast<uint32_t>(CTL1_Bits::PG);

    return program_halfword_to_bank(address, data, Timeout_Count, ctl_reg, pg_bit);
}

/**
 * @brief Reprograms a word in the FMC.
 *
 * This function reprograms a word in the FMC at the specified address. The
 * function determines which bank to use based on the address and the size of the
 * FMC. If the address is within the range of Bank 0, the function will use Bank 0
 * otherwise it will use Bank 1. The function waits until the reprogramming
 * operation is complete and then returns the state of the FMC.
 *
 * @param address The address to reprogram the word at.
 * @param data The data to be reprogrammed.
 * @return The state of the FMC after the reprogramming operation.
 */
auto FMC::reprogram_word(uint32_t address, uint32_t data) -> FMC_Error_Type {
    // Choose bank based on FMC size or address threshold
    bool is_bank0 = (get_fmc_size() <= Bank0_Size || address < Bank0_End_Address);

    // Pick the correct CTL register and PG bit
    auto ctl_reg = is_bank0 ? FMC_Regs::CTL0 : FMC_Regs::CTL1;
    // Unify PG bit to a uint32_t to avoid mixing enum classes
    uint32_t pg_bit = is_bank0
        ? static_cast<uint32_t>(CTL0_Bits::PG)
        : static_cast<uint32_t>(CTL1_Bits::PG);

    write_bit(*this, FMC_Regs::WSEN, static_cast<uint32_t>(WSEN_Bits::BPEN), true);

    return program_word_to_bank(address, data, Timeout_Count, ctl_reg, pg_bit);
}

/**
 * @brief Sets the wait state of the FMC.
 *
 * This function sets the wait state of the FMC to the specified value. The
 * wait state is used to control the amount of time the FMC waits after a
 * programming operation before doing another operation.
 *
 * @param wait The wait state to set the FMC to.
 */
void FMC::set_wait_state(Wait_State wait) {
    write_bit_range(*this, FMC_Regs::WS, static_cast<uint32_t>(WS_Bits::WSCNT), static_cast<uint32_t>(wait));
}

/**
 * @brief Gets the state of Bank 0 of the FMC.
 *
 * This function reads the state of Bank 0 of the FMC and returns it as a
 * FMC_Error_Type. The state can be one of the following values:
 *     - FMC_Error_Type::READY: Bank 0 is ready.
 *     - FMC_Error_Type::BUSY: Bank 0 is busy.
 *     - FMC_Error_Type::WP_ERROR: Bank 0 is in write protection error.
 *     - FMC_Error_Type::PG_ERROR: Bank 0 is in programming error.
 *
 * @return The state of Bank 0 of the FMC.
 */
auto FMC::get_bank0_state() -> FMC_Error_Type {
    if (read_bit(*this, FMC_Regs::STAT0, static_cast<uint32_t>(STAT0_Bits::BUSY))) {
        return FMC_Error_Type::BUSY;
    } else if (read_bit(*this, FMC_Regs::STAT0, static_cast<uint32_t>(STAT0_Bits::WPERR))) {
        return FMC_Error_Type::WP_ERROR;
    } else if (read_bit(*this, FMC_Regs::STAT0, static_cast<uint32_t>(STAT0_Bits::PGERR))) {
        return FMC_Error_Type::PG_ERROR;
    }

    return FMC_Error_Type::READY;
}

/**
 * @brief Gets the state of Bank 1 of the FMC.
 *
 * This function reads the state of Bank 1 of the FMC and returns it as a
 * FMC_Error_Type. The state can be one of the following values:
 *     - FMC_Error_Type::READY: Bank 1 is ready.
 *     - FMC_Error_Type::BUSY: Bank 1 is busy.
 *     - FMC_Error_Type::WP_ERROR: Bank 1 is in write protection error.
 *     - FMC_Error_Type::PG_ERROR: Bank 1 is in programming error.
 *
 * @return The state of Bank 1 of the FMC.
 */
auto FMC::get_bank1_state() -> FMC_Error_Type {
    if (read_bit(*this, FMC_Regs::STAT1, static_cast<uint32_t>(STAT1_Bits::BUSY))) {
        return FMC_Error_Type::BUSY;
    } else if (read_bit(*this, FMC_Regs::STAT1, static_cast<uint32_t>(STAT1_Bits::WPERR))) {
        return FMC_Error_Type::WP_ERROR;
    } else if (read_bit(*this, FMC_Regs::STAT1, static_cast<uint32_t>(STAT1_Bits::PGERR))) {
        return FMC_Error_Type::PG_ERROR;
    }

    return FMC_Error_Type::READY;
}

/**
 * @brief Waits for Bank 0 of the FMC to become ready.
 *
 * This function checks the state of Bank 0 of the FMC and waits until it is
 * no longer busy or the specified timeout is reached. If the timeout is reached
 * while Bank 0 is still busy, the state is set to FMC_Error_Type::TIMEOUT.
 *
 * @param timeout The maximum number of cycles to wait for Bank 0 to become ready.
 * @return The state of Bank 0 of the FMC, either ready, timeout, or an error state.
 */
auto FMC::ready_wait_bank0(uint32_t timeout) -> FMC_Error_Type {
    FMC_Error_Type state;

    // Loop until either we get a non-BUSY state or timeout occurs
    do {
        state = get_bank0_state();
    } while (state == FMC_Error_Type::BUSY && --timeout);

    // If we exited because of timeout, update the state
    if (timeout == 0 && state == FMC_Error_Type::BUSY) {
        return FMC_Error_Type::TIMEOUT;
    }

    return state;
}

/**
 * @brief Waits for Bank 1 of the FMC to become ready.
 *
 * This function checks the state of Bank 1 of the FMC and waits until it is
 * no longer busy or the specified timeout is reached. If the timeout is reached
 * while Bank 1 is still busy, the state is set to FMC_Error_Type::TIMEOUT.
 *
 * @param timeout The maximum number of cycles to wait for Bank 1 to become ready.
 * @return The state of Bank 1 of the FMC, either ready, timeout, or an error state.
 */
auto FMC::ready_wait_bank1(uint32_t timeout) -> FMC_Error_Type {
    FMC_Error_Type state;

    // Loop until either we get a non-BUSY state or timeout occurs
    do {
        state = get_bank1_state();
    } while (state == FMC_Error_Type::BUSY && --timeout);

    // If we exited because of timeout, update the state
    if (timeout == 0 && state == FMC_Error_Type::BUSY) {
        return FMC_Error_Type::TIMEOUT;
    }

    return state;
}

/**
 * @brief Retrieves the status of a specified flag in the FMC.
 *
 * This function returns the current status of a specified status flag
 * within the FMC module. It checks the flag's state using the internal
 * mechanism provided by get_value.
 *
 * @param flag The status flag to be checked, specified as a Status_Flags
 *             enumeration value.
 * @return True if the specified flag is set, otherwise false.
 */
auto FMC::get_flag(Status_Flags flag) -> bool {
    const auto& config = status_flag_index[static_cast<size_t>(flag)];
    return read_bit(*this, config.register_offset, static_cast<uint32_t>(config.bit_info));
}

/**
 * @brief Clears a specified status flag in the FMC.
 *
 * This function clears a given status flag specified by the Status_Flags
 * enumeration. The flag is cleared by writing a 0 to the corresponding bit
 * in the register containing the flag.
 *
 * @param flag The status flag to clear, specified as a Status_Flags enumeration
 *             value.
 */
void FMC::clear_flag(Status_Flags flag) {
    const auto& config = status_flag_index[static_cast<size_t>(flag)];
    write_bit(*this, config.register_offset, static_cast<uint32_t>(config.bit_info), true);
}

/**
 * @brief Retrieves the status of a specified interrupt flag in the FMC.
 *
 * This function checks the status of a given interrupt flag within the FMC
 * module. It checks the flag's state using the internal mechanism provided by
 * get_value.
 *
 * @param flag The interrupt flag to be checked, specified as an Interrupt_Flags
 *             enumeration value.
 * @return True if the specified flag is set, otherwise false.
 */
auto FMC::get_interrupt_flag(Interrupt_Flags flag) -> bool {
    const auto& config = interrupt_flag_index[static_cast<size_t>(flag)];
    const bool flag_value = read_bit(*this, config.flag_register_offset, static_cast<uint32_t>(config.flag_bit_info));
    const bool is_enabled = read_bit(*this, config.interrupt_register_offset, static_cast<uint32_t>(config.interrupt_bit_info));
    return (flag_value && is_enabled);
}

/**
 * @brief Clears a specified interrupt flag in the FMC.
 *
 * This function clears a given interrupt flag specified by the Interrupt_Flags
 * enumeration. The flag is cleared by writing a 1 to the corresponding bit in
 * the FMC's interrupt flag register.
 *
 * @param flag The interrupt flag to clear, specified as an Interrupt_Flags
 *             enumeration value.
 */
void FMC::clear_interrupt_flag(Interrupt_Flags flag) {
    const auto& config = interrupt_flag_index[static_cast<size_t>(flag)];
    write_bit(*this, config.flag_register_offset, static_cast<uint32_t>(config.flag_bit_info), true);
    write_bit(*this, config.interrupt_register_offset, static_cast<uint32_t>(config.interrupt_bit_info), true);
}

/**
 * @brief Enables or disables a specified interrupt type in the FMC.
 *
 * This function sets or clears a given interrupt type specified by the
 * Interrupt_Types enumeration. If the `enable` parameter is true, the
 * interrupt is enabled, and if the `enable` parameter is false, the interrupt
 * is disabled.
 *
 * @param type The interrupt type to enable or disable. Must be a value from the
 *             Interrupt_Types enumeration.
 * @param enable Set to true to enable the interrupt, false to disable it.
 */
void FMC::set_interrupt_enable(Interrupt_Types type, bool enable) {
    const auto& config = interrupt_type_index[static_cast<size_t>(type)];
    write_bit(*this, config.register_offset, static_cast<uint32_t>(config.bit_info), enable);
}

} // namespace fmc
