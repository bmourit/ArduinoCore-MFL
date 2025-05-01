//
// MFL gd32f30x EXMC peripheral register access in C++
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

#include "exmc_config.hpp"
#include "RegRW.hpp"

namespace exmc {

class EXMC {
public:
    static auto get_instance() -> EXMC&;

    // NOR and SRAM
    void nor_sram_reset(Block_Number block);
    void nor_sram_init();
    inline void nor_sram_configure(NOR_SRAM_Config config) {
        nor_sram_config_ = config;
        nor_sram_init();
    }
    void set_nor_sram_enable(Block_Number block, bool enable);

    // NAND
    void nand_reset(NPC_Block npc_block);
    void nand_init();
    inline void nand_configure(NAND_Config config) {
        nand_config_ = config;
        nand_init();
    }
    void set_nand_enable(NPC_Block npc_block, bool enable);

    // PC card
    void pccard_reset();
    void pccard_init();
    inline void pccard_configure(PCCARD_Config config) {
        pccard_config_ = config;
        pccard_init();
    }
    void set_pccard_enable(bool enable);

    // ECC
    void set_nor_sram_page_size(Block_Number block, Page_Size size);
    void set_nand_ecc_enable(NPC_Block npc_block, bool enable);
    auto get_ecc(NPC_Block npc_block) -> uint32_t;

    // Interrupts and flags
    auto get_flag(NPC_Block npc_block, Status_Flags flag) -> bool;
    void clear_flag(NPC_Block npc_block, Status_Flags flag);
    auto get_interrupt_flag(NPC_Block npc_block, Interrupt_Flags flag) -> bool;
    void clear_interrupt_flag(NPC_Block npc_block, Interrupt_Flags flag);
    void set_interrupt_enable(NPC_Block npc_block, Interrupt_Type type, bool enable);

    static inline constexpr uintptr_t EXMC_baseAddress = 0xA000'0000U;

    [[nodiscard]] inline auto reg_address(EXMC_Regs reg) const -> volatile uint32_t* {
        return reinterpret_cast<volatile uint32_t*>(EXMC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    EXMC();

    // Prevent copying or assigning
    EXMC(const EXMC&) = delete;
    auto operator=(const EXMC&) -> EXMC& = delete;

    NOR_SRAM_Config nor_sram_config_;
    NAND_Config nand_config_;
    PCCARD_Config pccard_config_;
    mutable bool is_clock_enabled_;

    inline auto get_snctl_offset(Block_Number block) -> EXMC_Regs {
        switch (block) {
            case Block_Number::BLOCK0: return EXMC_Regs::SNCTL0;
            case Block_Number::BLOCK1: return EXMC_Regs::SNCTL1;
            case Block_Number::BLOCK2: return EXMC_Regs::SNCTL2;
            case Block_Number::BLOCK3: return EXMC_Regs::SNCTL3;
            default: return EXMC_Regs::SNCTL0;
        }
    }

    inline auto get_sntcfg_offset(Block_Number block) -> EXMC_Regs {
        switch (block) {
            case Block_Number::BLOCK0: return EXMC_Regs::SNTCFG0;
            case Block_Number::BLOCK1: return EXMC_Regs::SNTCFG1;
            case Block_Number::BLOCK2: return EXMC_Regs::SNTCFG2;
            case Block_Number::BLOCK3: return EXMC_Regs::SNTCFG3;
            default: return EXMC_Regs::SNTCFG0;
        }
    }

    inline auto get_snwtcfg_offset(Block_Number block) -> EXMC_Regs {
        switch (block) {
            case Block_Number::BLOCK0: return EXMC_Regs::SNWTCFG0;
            case Block_Number::BLOCK1: return EXMC_Regs::SNWTCFG1;
            case Block_Number::BLOCK2: return EXMC_Regs::SNWTCFG2;
            case Block_Number::BLOCK3: return EXMC_Regs::SNWTCFG3;
            default: return EXMC_Regs::SNWTCFG0;
        }
    }

    inline auto get_npctl_offset(NPC_Block block) -> EXMC_Regs {
        switch (block) {
            case NPC_Block::NAND_BLOCK1: return EXMC_Regs::NPCTL1;
            case NPC_Block::NAND_BLOCK2: return EXMC_Regs::NPCTL2;
            case NPC_Block::PCCARD_BLOCK3: return EXMC_Regs::NPCTL3;
            default: return EXMC_Regs::NPCTL1;
        }
    }

    inline auto get_npinten_offset(NPC_Block block) -> EXMC_Regs {
        switch (block) {
            case NPC_Block::NAND_BLOCK1: return EXMC_Regs::NPINTEN1;
            case NPC_Block::NAND_BLOCK2: return EXMC_Regs::NPINTEN2;
            case NPC_Block::PCCARD_BLOCK3: return EXMC_Regs::NPINTEN3;
            default: return EXMC_Regs::NPINTEN1;
        }
    }

    inline auto get_npctcfg_offset(NPC_Block block) -> EXMC_Regs {
        switch (block) {
            case NPC_Block::NAND_BLOCK1: return EXMC_Regs::NPCTCFG1;
            case NPC_Block::NAND_BLOCK2: return EXMC_Regs::NPCTCFG2;
            case NPC_Block::PCCARD_BLOCK3: return EXMC_Regs::NPCTCFG3;
            default: return EXMC_Regs::NPCTCFG1;
        }
    }

    inline auto get_npatcfg_offset(NPC_Block block) -> EXMC_Regs {
        switch (block) {
            case NPC_Block::NAND_BLOCK1: return EXMC_Regs::NPATCFG1;
            case NPC_Block::NAND_BLOCK2: return EXMC_Regs::NPATCFG2;
            case NPC_Block::PCCARD_BLOCK3: return EXMC_Regs::NPATCFG3;
            default: return EXMC_Regs::NPATCFG1;
        }
    }

    inline auto get_necc_offset(NPC_Block npc_block) -> EXMC_Regs {
        switch (npc_block) {
            case NPC_Block::NAND_BLOCK1: return EXMC_Regs::NECC1;
            case NPC_Block::NAND_BLOCK2: return EXMC_Regs::NECC2;
            case NPC_Block::PCCARD_BLOCK3:
            default: return EXMC_Regs::NECC1;
        }
    }
};

} // namespace exmc
