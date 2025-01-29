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

FMC& FMC::get_instance() {
    static FMC instance;
    return instance;
}

FMC::FMC() {}

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
void FMC::lock(void) {
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
void FMC::lock_bank0(void) {
    write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::LK), true);
}

/**
 * @brief Lock the FMC bank1 memory region.
 * 
 * This function locks the FMC memory region of bank1 by setting the LK bit in
 * the FMC_CTL1 register. This prevents any further write operations to the
 * FMC bank1 memory region.
 */
void FMC::lock_bank1(void) {
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
FMC_Error_Type FMC::mass_erase() {
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);
    FMC_Error_Type state = ready_wait_bank0(timeout);

    if (state == FMC_Error_Type::READY) {
        write_bits_sequence(*this, FMC_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::MER), true,
                   static_cast<uint32_t>(CTL0_Bits::START), true);
        // Wait until ready
        timeout = reinterpret_cast<uint32_t>(Timeout_Count);
        state = ready_wait_bank0(timeout);
        write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::MER), false);
    }

    if (get_fmc_size() > Bank0_Size) {
        timeout = reinterpret_cast<uint32_t>(Timeout_Count);
        state = ready_wait_bank1(timeout);
        if (state == FMC_Error_Type::READY) {
            write_bits_sequence(*this, FMC_Regs::CTL1,
                       static_cast<uint32_t>(CTL1_Bits::MER), true,
                       static_cast<uint32_t>(CTL1_Bits::START), true);
            // Wait until ready
            timeout = reinterpret_cast<uint32_t>(Timeout_Count);
            state = ready_wait_bank1(timeout);
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
FMC_Error_Type FMC::erase_page(uint32_t address) {
    FMC_Error_Type state = FMC_Error_Type::READY;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);

    // Determine which bank to use based on the address and FMC size
    bool is_bank0 = (get_fmc_size() <= Bank0_Size || address < Bank0_End_Address);

    if (is_bank0) {
        state = erase_word_bank(address, timeout, FMC_Regs::CTL0, CTL0_Bits::PER,
                                CTL0_Bits::START, FMC_Regs::ADDR0);
    } else {
        state = erase_word_bank(address, timeout, FMC_Regs::CTL1, CTL1_Bits::PER,
                                CTL1_Bits::START, FMC_Regs::ADDR1);
    }

    return state;
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
FMC_Error_Type FMC::erase_bank0() {
    FMC_Error_Type state = FMC_Error_Type::READY;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);

    state = ready_wait_bank0(timeout);
    if (state == FMC_Error_Type::READY) {
        write_bits_sequence(*this, FMC_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::MER), true,
                   static_cast<uint32_t>(CTL0_Bits::START), true);
        // Wait until ready
        timeout = reinterpret_cast<uint32_t>(Timeout_Count);
        state = ready_wait_bank0(timeout);
        write_bit(*this, FMC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::MER), false);
    }

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
FMC_Error_Type FMC::erase_bank1() {
    FMC_Error_Type state = FMC_Error_Type::READY;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);

    state = ready_wait_bank1(timeout);
    if (state == FMC_Error_Type::READY) {
        write_bits_sequence(*this, FMC_Regs::CTL1,
                   static_cast<uint32_t>(CTL1_Bits::MER), true,
                   static_cast<uint32_t>(CTL1_Bits::START), true);
        // Wait until ready
        timeout = reinterpret_cast<uint32_t>(Timeout_Count);
        state = ready_wait_bank1(timeout);
        write_bit(*this, FMC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::MER), false);
    }

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
FMC_Error_Type FMC::program_word(uint32_t address, uint32_t data) {
    FMC_Error_Type state = FMC_Error_Type::READY;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);

    // Determine which bank to use based on the address and FMC size
    bool is_bank0 = (get_fmc_size() <= Bank0_Size || address < Bank0_End_Address);

    if (is_bank0) {
        state = program_word_to_bank(address, data, timeout,
                                     FMC_Regs::CTL0, CTL0_Bits::PG);
    } else {
        state = program_word_to_bank(address, data, timeout,
                                     FMC_Regs::CTL1, CTL1_Bits::PG);
    }

    return state;
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
FMC_Error_Type FMC::program_halfword(uint32_t address, uint16_t data) {
    FMC_Error_Type state = FMC_Error_Type::READY;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);

    // Determine which bank to use based on the address and FMC size
    bool is_bank0 = (get_fmc_size() <= Bank0_Size || address < Bank0_End_Address);

    if (is_bank0) {
        state = program_halfword_to_bank(address, data, timeout,
                                         FMC_Regs::CTL0, CTL0_Bits::PG);
    } else {
        state = program_halfword_to_bank(address, data, timeout,
                                         FMC_Regs::CTL1, CTL1_Bits::PG);
    }

    return state;
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
FMC_Error_Type FMC::reprogram_word(uint32_t address, uint32_t data) {
    FMC_Error_Type state = FMC_Error_Type::READY;
    uint32_t timeout = reinterpret_cast<uint32_t>(Timeout_Count);

    // Determine which bank to use based on the address and FMC size
    bool is_bank0 = (get_fmc_size() <= Bank0_Size || address < Bank0_End_Address);

    if (is_bank0) {
        write_bit(*this, FMC_Regs::WSEN, static_cast<uint32_t>(WSEN_Bits::BPEN), true);
        state = program_word_to_bank(address, data, timeout,
                                     FMC_Regs::CTL0, CTL0_Bits::PG);
    } else {
        write_bit(*this, FMC_Regs::WSEN, static_cast<uint32_t>(WSEN_Bits::BPEN), true);
        state = program_word_to_bank(address, data, timeout,
                                     FMC_Regs::CTL1, CTL1_Bits::PG);
    }

    return state;
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
FMC_Error_Type FMC::get_bank0_state() {
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
FMC_Error_Type FMC::get_bank1_state() {
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
 * @brief Waits for Bank 0 of the FMC to become ready.
 * 
 * This function checks the state of Bank 0 of the FMC and waits until it is
 * no longer busy or the specified timeout is reached. If the timeout is reached
 * while Bank 0 is still busy, the state is set to FMC_Error_Type::TIMEOUT.
 * 
 * @param timeout The maximum number of cycles to wait for Bank 0 to become ready.
 * @return The state of Bank 0 of the FMC, either ready, timeout, or an error state.
 */
FMC_Error_Type FMC::ready_wait_bank0(uint32_t timeout) {
    FMC_Error_Type state = FMC_Error_Type::BUSY;

    do {
        state = get_bank0_state();
        timeout = timeout - 1;
    } while ((state == FMC_Error_Type::BUSY) && (timeout != 0));

    if (state == FMC_Error_Type::BUSY) {
        state = FMC_Error_Type::TIMEOUT;
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
FMC_Error_Type FMC::ready_wait_bank1(uint32_t timeout) {
    FMC_Error_Type state = FMC_Error_Type::BUSY;

    do {
        state = get_bank1_state();
        timeout = timeout - 1;
    } while ((state == FMC_Error_Type::BUSY) && (timeout != 0));

    if (state == FMC_Error_Type::BUSY) {
        state = FMC_Error_Type::TIMEOUT;
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
bool FMC::get_flag(Status_Flags flag) {
    return get_value(flag);
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
    set_value(flag, Set);
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
bool FMC::get_interrupt_flag(Interrupt_Flags flag) {
    return get_value(flag);
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
    set_value(flag, Set);
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
    set_value(type, enable ? Set : Clear);
}

/**
 * @brief Programs a word to a specified address in either bank 0 or bank 1 of the
 * FMC. The function waits for the programming operation to complete and
 * returns the resulting state of the FMC.
 *
 * @param address The address in either bank 0 or bank 1 where the data will be
 *                written.
 * @param data The 32-bit data word to be written to the specified address.
 * @param timeout The timeout value in milliseconds to wait for the FMC to
 *                become ready after the programming operation is started.
 * @param control_reg The control register to be used for programming the
 *                    specified bank. Must be one of the FMC_Regs enumeration
 *                    values FMC_Regs::CTL0 or FMC_Regs::CTL1.
 * @param program_bit The bit position in the control register to be used to
 *                    control the programming operation. Must be one of the
 *                    FMC_Program_Bit enumeration values.
 *
 * @return The resulting state of the FMC after the programming operation has
 *         completed. Will be one of the FMC_Error_Type enumeration values.
 */
template<typename T>
FMC_Error_Type FMC::program_word_to_bank(uint32_t address, uint32_t data,
        uint32_t timeout, FMC_Regs control_reg, T program_bit) {
    FMC_Error_Type state = FMC_Error_Type::READY;

    if (control_reg == FMC_Regs::CTL0) {
        state = ready_wait_bank0(timeout);
    } else {
        state = ready_wait_bank1(timeout);
    }

    if (state == FMC_Error_Type::READY) {
        // Set the programming bit
        write_bit(*this, control_reg, static_cast<uint32_t>(program_bit), true);

        // Write the data to the specified address
        *reinterpret_cast<volatile uint32_t*>(address) = data;

        // Wait until programming completes
        if (control_reg == FMC_Regs::CTL0) {
            state = ready_wait_bank0(timeout);
        } else {
            state = ready_wait_bank1(timeout);
        }

        // Clear the programming bit
        write_bit(*this, control_reg, static_cast<uint32_t>(program_bit), false);
    }

    return state;
}

/**
 * @brief Programs a halfword to a specified address in either bank 0 or bank 1 of the
 * FMC. The function waits for the programming operation to complete and
 * returns the resulting state of the FMC.
 *
 * @param address The address in either bank 0 or bank 1 where the data will be
 *                written.
 * @param data The 16-bit data halfword to be written to the specified address.
 * @param timeout The timeout value in milliseconds to wait for the FMC to
 *                become ready after the programming operation is started.
 * @param control_reg The control register to be used for programming the
 *                    specified bank. Must be one of the FMC_Regs enumeration
 *                    values FMC_Regs::CTL0 or FMC_Regs::CTL1.
 * @param program_bit The bit position in the control register to be used to
 *                    control the programming operation. Must be one of the
 *                    FMC_Program_Bit enumeration values.
 *
 * @return The resulting state of the FMC after the programming operation has
 *         completed. Will be one of the FMC_Error_Type enumeration values.
 */
template<typename T>
FMC_Error_Type FMC::program_halfword_to_bank(uint32_t address, uint16_t data,
        uint32_t timeout, FMC_Regs control_reg, T program_bit) {
    FMC_Error_Type state = FMC_Error_Type::READY;

    if (control_reg == FMC_Regs::CTL0) {
        state = ready_wait_bank0(timeout);
    } else {
        state = ready_wait_bank1(timeout);
    }

    if (state == FMC_Error_Type::READY) {
        // Set the programming bit
        write_bit(*this, control_reg, static_cast<uint32_t>(program_bit), true);

        // Write the data to the specified address
        *reinterpret_cast<volatile uint16_t*>(address) = data;

        // Wait until programming completes
        if (control_reg == FMC_Regs::CTL0) {
            state = ready_wait_bank0(timeout);
        } else {
            state = ready_wait_bank1(timeout);
        }

        // Clear the programming bit
        write_bit(*this, control_reg, static_cast<uint32_t>(program_bit), false);
    }

    return state;
}

/**
 * @brief Erases a word at a specified address in either bank 0 or bank 1 of the
 * FMC. The function waits for the erase operation to complete and
 * returns the resulting state of the FMC.
 *
 * @param address The address in either bank 0 or bank 1 where the data will be
 *                erased.
 * @param timeout The timeout value in milliseconds to wait for the FMC to
 *                become ready after the erase operation is started.
 * @param control_reg The control register to be used for erasing the
 *                    specified bank. Must be one of the FMC_Regs enumeration
 *                    values FMC_Regs::CTL0 or FMC_Regs::CTL1.
 * @param erase_bit The bit position in the control register to be used to
 *                    control the erase operation. Must be one of the
 *                    FMC_Erase_Bit enumeration values.
 * @param start_bit The bit position in the control register to be used to
 *                    start the erase operation. Must be one of the
 *                    FMC_Start_Bit enumeration values.
 * @param address_reg The address register to be used for erasing the
 *                    specified bank. Must be one of the FMC_Regs enumeration
 *                    values FMC_Regs::ADDR0 or FMC_Regs::ADDR1.
 *
 * @return The resulting state of the FMC after the erase operation has
 *         completed. Will be one of the FMC_Error_Type enumeration values.
 */
template<typename T>
FMC_Error_Type FMC::erase_word_bank(uint32_t address, uint32_t timeout,
               FMC_Regs control_reg, T erase_bit, T start_bit, FMC_Regs address_reg) {
    FMC_Error_Type state = FMC_Error_Type::READY;

    state = (control_reg == FMC_Regs::CTL0) ? ready_wait_bank0(timeout) : ready_wait_bank1(timeout);

    if (state == FMC_Error_Type::READY) {
        // Set the programming bit
        write_bit(*this, control_reg, static_cast<uint32_t>(erase_bit), true);
        write_register(*this, address_reg, address);
        if (control_reg == FMC_Regs::CTL1) {
            if (read_bit(*this, FMC_Regs::OBSTAT, static_cast<uint32_t>(OBSTAT_Bits::SPC))) {
                write_register(*this, FMC_Regs::ADDR0, address);
            }
        }
        // Write the start bit
        write_bit(*this, control_reg, static_cast<uint32_t>(start_bit), true);

        // Wait until erase completes
        state = (control_reg == FMC_Regs::CTL0) ? ready_wait_bank0(timeout) : ready_wait_bank1(timeout);

        // Clear the erase bit
        write_bit(*this, control_reg, static_cast<uint32_t>(erase_bit), false);
    }

    return state;
}


} // namespace fmc
