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

#pragma once

#include <cstdint>

#include "fmc_config.hpp"
#include "RegRW.hpp"

namespace fmc {

class FMC {
public:
    static auto get_instance() -> FMC&;

    // Lock
    void lock();
    void lock_bank0();
    void lock_bank1();

    // Unlock
    void unlock();
    void unlock_bank0();
    void unlock_bank1();

    // Erase
    auto mass_erase() -> FMC_Error_Type;
    auto erase_page(uint32_t address) -> FMC_Error_Type;
    auto erase_bank0() -> FMC_Error_Type;
    auto erase_bank1() -> FMC_Error_Type;

    // Program
    auto program_word(uint32_t address, uint32_t data) -> FMC_Error_Type;
    auto program_halfword(uint32_t address, uint16_t data) -> FMC_Error_Type;
    auto reprogram_word(uint32_t address, uint32_t data) -> FMC_Error_Type;

    // Delay
    void set_wait_state(Wait_State wait);

    // State
    auto get_bank0_state() -> FMC_Error_Type;
    auto get_bank1_state() -> FMC_Error_Type;

    // Wait
    auto ready_wait_bank0(uint32_t timeout) -> FMC_Error_Type;
    auto ready_wait_bank1(uint32_t timeout) -> FMC_Error_Type;

    // Flags
    auto get_flag(Status_Flags flag) -> bool;
    void clear_flag(Status_Flags flag);

    // Interruppt flags
    auto get_interrupt_flag(Interrupt_Flags flag) -> bool;
    void clear_interrupt_flag(Interrupt_Flags flag);

    // Interrupts
    void set_interrupt_enable(Interrupt_Types type, bool enable);

    // Base address
    static inline constexpr uintptr_t FMC_baseAddress = 0x4002'2000U;

    // Register address
    [[nodiscard]] inline auto reg_address(FMC_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(FMC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    FMC();

    // Prevent copying or assigning
    FMC(const FMC&) = delete;
    auto operator=(const FMC&) -> FMC& = delete;

    // Inlined methods
    template<typename T>
    inline auto program_word_to_bank(
        uint32_t address, uint32_t data,
        uint32_t timeout, FMC_Regs control_reg, T program_bit) -> FMC_Error_Type
    {
        auto wait_fn = [this, control_reg](uint32_t to) -> FMC_Error_Type {
            return (control_reg == FMC_Regs::CTL0)
                   ? this->ready_wait_bank0(to)
                   : this->ready_wait_bank1(to);
        };

        FMC_Error_Type state = wait_fn(timeout);
        if (state != FMC_Error_Type::READY) return state;

        const auto bit = static_cast<uint32_t>(program_bit);
        write_bit(*this, control_reg, bit, true);
        *reinterpret_cast<volatile uint32_t*>(address) = data;

        state = wait_fn(timeout);
        write_bit(*this, control_reg, bit, false);

        return state;
    }

    template<typename T>
    inline auto program_halfword_to_bank(
        uint32_t address, uint16_t data, uint32_t timeout,
        FMC_Regs control_reg, T program_bit) -> FMC_Error_Type
    {
        auto wait_fn = [this, control_reg](uint32_t to) -> FMC_Error_Type {
            return (control_reg == FMC_Regs::CTL0)
                ? this->ready_wait_bank0(to)
                : this->ready_wait_bank1(to);
        };

        FMC_Error_Type state = wait_fn(timeout);
        if (state != FMC_Error_Type::READY) return state;

        const auto bit = static_cast<uint32_t>(program_bit);
        write_bit(*this, control_reg, bit, true);
        *reinterpret_cast<volatile uint16_t*>(address) = data;

        state = wait_fn(timeout);
        write_bit(*this, control_reg, bit, false);

        return state;
    }

    template<typename T>
    inline auto erase_word_bank(
        uint32_t address, uint32_t timeout,
        FMC_Regs control_reg, T erase_bit, T start_bit,
        FMC_Regs address_reg) -> FMC_Error_Type
    {
        auto wait_fn = [this, control_reg](uint32_t to) -> FMC_Error_Type {
            return (control_reg == FMC_Regs::CTL0)
                ? this->ready_wait_bank0(to)
                : this->ready_wait_bank1(to);
        };

        FMC_Error_Type state = wait_fn(timeout);
        if (state != FMC_Error_Type::READY) return state;

        const auto e_bit = static_cast<uint32_t>(erase_bit);
        const auto s_bit = static_cast<uint32_t>(start_bit);

        write_bit(*this, control_reg, e_bit, true);
        write_register(*this, address_reg, address);

        if (control_reg == FMC_Regs::CTL1 &&
            read_bit(*this, FMC_Regs::OBSTAT, static_cast<uint32_t>(OBSTAT_Bits::SPC))) {
            write_register(*this, FMC_Regs::ADDR0, address);
        }

        // Write the start bit
        write_bit(*this, control_reg, s_bit, true);

        // Wait again and clear the erase bit
        state = wait_fn(timeout);
        write_bit(*this, control_reg, e_bit, false);

        return state;
    }

    inline auto get_fmc_size() -> uint16_t {
        return *reinterpret_cast<const uint16_t*>(Flash_Size_Addess);
    }
};

} // namespace fmc
