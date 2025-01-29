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

#include <stdint.h>

#include "fmc_config.hpp"
#include "RegRW.hpp"

namespace fmc {

class RCU;

class FMC {
public:
    static FMC& get_instance();

    // Lock
    void lock();
    void lock_bank0();
    void lock_bank1();
    // Unlock
    void unlock();
    void unlock_bank0();
    void unlock_bank1();
    // Erase
    FMC_Error_Type mass_erase();
    FMC_Error_Type erase_page(uint32_t address);
    FMC_Error_Type erase_bank0();
    FMC_Error_Type erase_bank1();
    // Program
    FMC_Error_Type program_word(uint32_t address, uint32_t data);
    FMC_Error_Type program_halfword(uint32_t address, uint16_t data);
    FMC_Error_Type reprogram_word(uint32_t address, uint32_t data);
    // Delay
    void set_wait_state(Wait_State wait);
    // State
    FMC_Error_Type get_bank0_state();
    FMC_Error_Type get_bank1_state();
    // Wait
    FMC_Error_Type ready_wait_bank0(uint32_t timeout);
    FMC_Error_Type ready_wait_bank1(uint32_t timeout);
    // Flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Status_Flags flag);
    // Interruppt flags
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Interrupt_Flags flag);
    // Interrupts
    void set_interrupt_enable(Interrupt_Types type, bool enable);

   // Base address
    static inline constexpr uintptr_t FMC_baseAddress = 0x4002200U;

    // Register address
    inline volatile uint32_t* reg_address(FMC_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(FMC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    FMC();

    // Prevent copying or assigning
    FMC(const FMC&) = delete;
    FMC& operator=(const FMC&) = delete;

    template<typename T>
    inline FMC_Error_Type program_word_to_bank(uint32_t address, uint32_t data,
                uint32_t timeout, FMC_Regs control_reg, T program_bit);

    template<typename T>
    inline FMC_Error_Type program_halfword_to_bank(uint32_t address, uint16_t data,
                uint32_t timeout, FMC_Regs control_reg, T program_bit);

    template<typename T>
    inline FMC_Error_Type erase_word_bank(uint32_t address, uint32_t timeout,
                FMC_Regs control_reg, T erase_bit, T start_bit, FMC_Regs address_reg);

    inline uint16_t get_fmc_size() {
        return *reinterpret_cast<const uint16_t*>(Flash_Size_Addess);
    }

    inline bool get_value(Interrupt_Flags flag) {
        const auto& info = interrupt_flag_index[static_cast<size_t>(flag)];
        uint32_t flag_value = read_register<uint32_t>(*this, info.flag_register_offset);
        uint32_t interrupt_value = read_register<uint32_t>(*this, info.interrupt_register_offset);

        const uint32_t flag_width = info.flag_bit_info & 0xFFU;
        const uint32_t flag_bitno = info.flag_bit_info >> 16U;
        const uint32_t interrupt_width = info.interrupt_bit_info & 0xFFU;
        const uint32_t interrupt_bitno = info.interrupt_bit_info >> 16U;

        flag_value >>= flag_bitno;
        flag_value &= ((1U << flag_width) - 1U);
        interrupt_value >>= interrupt_bitno;
        interrupt_value &= ((1U << interrupt_width) - 1U);

        return (flag_value && interrupt_value);
    }

    inline bool get_value(Status_Flags flag) {
        const auto& info = status_flag_index[static_cast<size_t>(flag)];
        uint32_t flag_value = read_register<uint32_t>(*this, info.register_offset);

        const uint32_t flag_width = info.bit_info & 0xFFU;
        const uint32_t flag_bitno = info.bit_info >> 16U;

        flag_value >>= flag_bitno;
        flag_value &= ((1U << flag_width) - 1U);

        return flag_value;
    }

    inline void set_value(Interrupt_Flags flag, uint32_t value) {
        const auto& info = interrupt_flag_index[static_cast<size_t>(flag)];
        uint32_t flag_value = read_register<uint32_t>(*this, info.flag_register_offset);
        uint32_t interrupt_value = read_register<uint32_t>(*this, info.interrupt_register_offset);

        const uint32_t flag_width = info.flag_bit_info & 0xFFU;
        const uint32_t flag_bitno = info.flag_bit_info >> 16U;
        const uint32_t interrupt_width = info.interrupt_bit_info & 0xFFU;
        const uint32_t interrupt_bitno = info.interrupt_bit_info >> 16U;

        flag_value &= ~(((1U << flag_width) - 1U) << flag_bitno);
        flag_value |= value << flag_bitno;
        interrupt_value &= ~(((1U << interrupt_width) - 1U) << interrupt_bitno);
        interrupt_value |= value << interrupt_bitno;

        write_register(*this, info.flag_register_offset, flag_value);
        write_register(*this, info.interrupt_register_offset, interrupt_value);
    }

    inline void set_value(Status_Flags flag, uint32_t value) {
        const auto& info = status_flag_index[static_cast<size_t>(flag)];
        uint32_t flag_value = read_register<uint32_t>(*this, info.register_offset);

        const uint32_t flag_width = info.bit_info & 0xFFU;
        const uint32_t flag_bitno = info.bit_info >> 16U;

        flag_value &= ~(((1U << flag_width) - 1U) << flag_bitno);
        flag_value |= value << flag_bitno;

        write_register(*this, info.register_offset, flag_value);
    }

    inline void set_value(Interrupt_Types type, uint32_t value) {
        const auto& info = interrupt_type_index[static_cast<size_t>(type)];
        uint32_t type_value = read_register<uint32_t>(*this, info.register_offset);

        const uint32_t type_width = info.bit_info & 0xFFU;
        const uint32_t type_bitno = info.bit_info >> 16U;

        type_value &= ~(((1U << type_width) - 1U) << type_bitno);
        type_value |= value << type_bitno;

        write_register(*this, info.register_offset, type_value);
    }
};

} // namespace fmc
