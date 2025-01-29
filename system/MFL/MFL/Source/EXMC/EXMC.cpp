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

#include "EXMC.hpp"
#include "RCU.hpp"

namespace exmc {

EXMC& EXMC::get_instance() {
    static EXMC instance;
    return instance;
}

EXMC::EXMC() : nor_sram_config_(nor_sram_default_config),
    nand_config_(nand_default_config),
    pccard_config_(pccard_default_config),
    is_clock_enabled_(false)
{
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_EXMC, true);
        is_clock_enabled_ = true;
    }
}

/**
 * @brief Reset the NOR/SRAM block given by `block`.
 * 
 * This function will reset the SNCTL, SNTCFG and SNWTCFG registers for the given block
 * to their default values. No other registers are affected.
 * 
 * @param block The block number to reset.
 */
void EXMC::nor_sram_reset(Block_Number block) {
    EXMC_Regs snctl_offset = get_snctl_offset(block);
    EXMC_Regs sntcfg_offset = get_sntcfg_offset(block);
    EXMC_Regs snwtcfg_offset = get_snwtcfg_offset(block);

    write_register(*this, snctl_offset, (block == Block_Number::BLOCK0) ? SNCTL_Block0_Reset : SNCTL_Block1_2_3_Reset);
    write_register(*this, sntcfg_offset, Common_Reset);
    write_register(*this, snwtcfg_offset, Common_Reset);
}

/**
 * @brief Initialize the NOR/SRAM block configuration registers.
 * 
 * This function sets up the NOR/SRAM block by configuring the SNCTL and SNTCFG
 * registers based on the settings provided in the `nor_sram_config_` structure.
 * It handles both basic and extended mode configurations, setting the necessary
 * bit fields and ranges for asynchronous and synchronous operations.
 * 
 * The function writes the timing parameters, control settings, and mode configurations
 * to the respective registers, enabling features like address multiplexing, burst mode,
 * write enable, and asynchronous wait, according to the configuration.
 * 
 * In extended mode, it additionally configures the SNWTCFG register with the write timing
 * parameters. If not in extended mode, the function will reset the register to default.
 */
void EXMC::nor_sram_init() {
    EXMC_Regs reg_offset = get_snctl_offset(nor_sram_config_.block);
    
    write_bits_sequence(*this, reg_offset,
               static_cast<uint32_t>(SNCTLX_Bits::NRMUX), nor_sram_config_.address_mux,
               static_cast<uint32_t>(SNCTLX_Bits::NREN), false,
               static_cast<uint32_t>(SNCTLX_Bits::SBRSTEN), nor_sram_config_.burst,
               static_cast<uint32_t>(SNCTLX_Bits::NRWTPOL), (nor_sram_config_.polarity == Signal_Polarity::HIGH),
               static_cast<uint32_t>(SNCTLX_Bits::WRAPEN), nor_sram_config_.wrap,
               static_cast<uint32_t>(SNCTLX_Bits::NRWTCFG), (nor_sram_config_.nwait_active == NWAIT_Active::DURING),
               static_cast<uint32_t>(SNCTLX_Bits::WREN), nor_sram_config_.memory_write,
               static_cast<uint32_t>(SNCTLX_Bits::NRWTEN), nor_sram_config_.nwait_signal,
               static_cast<uint32_t>(SNCTLX_Bits::EXMODEN), nor_sram_config_.extended_mode,
               static_cast<uint32_t>(SNCTLX_Bits::ASYNCWAIT), nor_sram_config_.async_wait,
               static_cast<uint32_t>(SNCTLX_Bits::SYNCWR), (nor_sram_config_.mode == Write_Mode::SYNC));
    write_bit_ranges(*this, reg_offset,
               static_cast<uint32_t>(SNCTLX_Bits::NRTP), static_cast<uint32_t>(nor_sram_config_.type),
               static_cast<uint32_t>(SNCTLX_Bits::NRW), static_cast<uint32_t>(nor_sram_config_.width));

    write_bit_ranges(*this, reg_offset,
               static_cast<uint32_t>(SNTCFGX_Bits::ASET), nor_sram_config_.rw_timing->async_ast - 1,
               static_cast<uint32_t>(SNTCFGX_Bits::AHLD), nor_sram_config_.rw_timing->async_aht - 1,
               static_cast<uint32_t>(SNTCFGX_Bits::DSET), nor_sram_config_.rw_timing->async_dst - 1,
               static_cast<uint32_t>(SNTCFGX_Bits::BUSLAT), nor_sram_config_.rw_timing->bus_latency - 1,
               static_cast<uint32_t>(SNTCFGX_Bits::CKDIV), static_cast<uint32_t>(nor_sram_config_.rw_timing->divider),
               static_cast<uint32_t>(SNTCFGX_Bits::DLAT), static_cast<uint32_t>(nor_sram_config_.rw_timing->sync_latency),
               static_cast<uint32_t>(SNTCFGX_Bits::ASYNCMOD), static_cast<uint32_t>(nor_sram_config_.rw_timing->async_access));
    write_bit(*this, reg_offset, static_cast<uint32_t>(SNCTLX_Bits::NREN), (nor_sram_config_.type == Memory_Type::NOR));

    if (nor_sram_config_.extended_mode == true) {
        write_bit_ranges(*this, reg_offset,
                   static_cast<uint32_t>(SNWTCFGX_Bits::WASET), nor_sram_config_.write_timing->async_ast - 1,
                   static_cast<uint32_t>(SNWTCFGX_Bits::WAHLD), nor_sram_config_.write_timing->async_aht - 1,
                   static_cast<uint32_t>(SNWTCFGX_Bits::WDSET), nor_sram_config_.write_timing->async_dst - 1,
                   static_cast<uint32_t>(SNWTCFGX_Bits::WBUSLAT), nor_sram_config_.write_timing->bus_latency - 1,
                   static_cast<uint32_t>(SNWTCFGX_Bits::WASET), static_cast<uint32_t>(nor_sram_config_.write_timing->async_access));
    } else {
        write_register(*this, reg_offset, Common_Reset);
    }
}

/**
 * \brief Enable or disable NOR/PSRAM block
 * \param block Which block to enable or disable
 * \param enable True to enable, false to disable
 */
void EXMC::set_nor_sram_enable(Block_Number block, bool enable) {
    EXMC_Regs snctl_offset = get_snctl_offset(block);
    write_bit(*this, snctl_offset, static_cast<uint32_t>(SNCTLX_Bits::NRBKEN), enable);
}

/**
 * \brief Resets the NAND Flash block
 * \param block Which block to reset
 */
void EXMC::nand_reset(NPC_Block block) {
    EXMC_Regs npctl_offset = get_npctl_offset(block);
    EXMC_Regs npinten_offset = get_npinten_offset(block);
    EXMC_Regs npctcfg_offset = get_npctcfg_offset(block);
    EXMC_Regs npatcfg_offset = get_npatcfg_offset(block);

    write_register(*this, npctl_offset, NPCTL_Block1_2_Reset);
    write_register(*this, npinten_offset, NPINTEN_Block1_2_Reset);
    write_register(*this, npctcfg_offset, Common_Block3_Reset);
    write_register(*this, npatcfg_offset, Common_Block3_Reset);
}

/**
 * @brief Initializes the NAND Flash block configuration registers.
 * 
 * This function configures the NAND Flash block by setting up the NPCTL,
 * NPCTCFG, and NPATCFG registers based on the settings provided in the
 * `nand_config_` structure. It enables or disables features such as wait
 * signal and ECC, and configures timing parameters like databus width,
 * ECC size, counter latency, and attribute latency.
 * 
 * The function writes the timing parameters, control settings, and mode
 * configurations to the respective registers using helper functions to
 * manipulate individual bits and bit ranges.
 */
void EXMC::nand_init() {
    EXMC_Regs npctl_offset = get_npctl_offset(nand_config_.npc_block);

    write_bits_sequence(*this, npctl_offset,
               static_cast<uint32_t>(NPCTLX_Bits::NDWTEN), nand_config_.wait,
               static_cast<uint32_t>(NPCTLX_Bits::NDTP), true,
               static_cast<uint32_t>(NPCTLX_Bits::ECCEN), nand_config_.ecc);
    write_bit_ranges(*this, npctl_offset,
               static_cast<uint32_t>(NPCTLX_Bits::NDW), static_cast<uint32_t>(nand_config_.databus_width),
               static_cast<uint32_t>(NPCTLX_Bits::ECCSZ), static_cast<uint32_t>(nand_config_.ecc_size),
               static_cast<uint32_t>(NPCTLX_Bits::CTR), static_cast<uint32_t>(nand_config_.ctr_latency),
               static_cast<uint32_t>(NPCTLX_Bits::ATR), static_cast<uint32_t>(nand_config_.atr_latency));

    EXMC_Regs npctcfg_offset = get_npctcfg_offset(nand_config_.npc_block);

    write_bit_ranges(*this, npctcfg_offset,
               static_cast<uint32_t>(NPCTCFGX_Bits::COMSET), nand_config_.common_timing->st - 1,
               static_cast<uint32_t>(NPCTCFGX_Bits::COMWAIT), nand_config_.common_timing->wt - 1,
               static_cast<uint32_t>(NPCTCFGX_Bits::COMHLD), nand_config_.common_timing->ht,
               static_cast<uint32_t>(NPCTCFGX_Bits::COMHIZ), nand_config_.common_timing->dbhzt - 1);

    EXMC_Regs npatcfg_offset = get_npatcfg_offset(nand_config_.npc_block);

    write_bit_ranges(*this, npatcfg_offset,
               static_cast<uint32_t>(NPATCFGX_Bits::ATTSET), nand_config_.attribute_timing->st - 1,
               static_cast<uint32_t>(NPATCFGX_Bits::ATTWAIT), nand_config_.attribute_timing->wt - 1,
               static_cast<uint32_t>(NPATCFGX_Bits::ATTHLD), nand_config_.attribute_timing->ht,
               static_cast<uint32_t>(NPATCFGX_Bits::ATTHIZ), nand_config_.attribute_timing->dbhzt - 1);
}

/**
 * @brief Enable or disable a NAND Flash block.
 *
 * This function sets the NDBKEN bit in the NPCTL register to enable or
 * disable the specified NAND Flash block.
 *
 * @param npc_block The NAND Flash block to enable or disable.
 * @param enable Set to true to enable the block, or false to disable it.
 */
void EXMC::set_nand_enable(NPC_Block npc_block, bool enable) {
    EXMC_Regs npctl_offset = get_npctl_offset(npc_block);
    write_bit(*this, npctl_offset, static_cast<uint32_t>(NPCTLX_Bits::NDBKEN), enable);
}

/**
 * \brief Resets the PC Card block configuration registers.
 *
 * This function resets the PC Card block by writing the default reset values
 * to the NPCTL3, NPINTEN3, NPCTCFG3, NPATCFG3, and PIOTCFG3 registers.
 * It ensures that the PC Card block is returned to its default state,
 * clearing any previous configurations.
 */
void EXMC::pccard_reset() {
    write_register(*this, EXMC_Regs::NPCTL3, NPCTL_Block3_Reset);
    write_register(*this, EXMC_Regs::NPINTEN3, NPINTEN_Block3_Reset);
    write_register(*this, EXMC_Regs::NPCTCFG3, Common_Block3_Reset);
    write_register(*this, EXMC_Regs::NPATCFG3, Common_Block3_Reset);
    write_register(*this, EXMC_Regs::PIOTCFG3, Common_Block3_Reset);
}

/**
 * @brief Initializes the PC Card block configuration registers.
 *
 * This function initializes the PC Card block configuration registers with
 * the values specified in the PCCARD_Config struct. This function should be
 * called after the pccard_reset() function to ensure that the PC Card block
 * is properly configured.
 */
void EXMC::pccard_init() {
    write_bit(*this, EXMC_Regs::NPCTL3,
               static_cast<uint32_t>(NPCTLX_Bits::NDWTEN), pccard_config_.wait);

    write_bit_ranges(*this, EXMC_Regs::NPCTL3,
               static_cast<uint32_t>(NPCTLX_Bits::NDW), static_cast<uint32_t>(Bus_Width::WIDTH_16BITS),
               static_cast<uint32_t>(NPCTLX_Bits::CTR), static_cast<uint32_t>(pccard_config_.ctr_latency),
               static_cast<uint32_t>(NPCTLX_Bits::ATR), static_cast<uint32_t>(pccard_config_.atr_latency));

    write_bit_ranges(*this, EXMC_Regs::NPCTCFG3,
               static_cast<uint32_t>(NPCTCFGX_Bits::COMSET), pccard_config_.common_timing->st - 1,
               static_cast<uint32_t>(NPCTCFGX_Bits::COMWAIT), pccard_config_.common_timing->wt - 1,
               static_cast<uint32_t>(NPCTCFGX_Bits::COMHLD), pccard_config_.common_timing->ht,
               static_cast<uint32_t>(NPCTCFGX_Bits::COMHIZ), pccard_config_.common_timing->dbhzt - 1);

    write_bit_ranges(*this, EXMC_Regs::NPATCFG3,
               static_cast<uint32_t>(NPATCFGX_Bits::ATTSET), pccard_config_.attribute_timing->st - 1,
               static_cast<uint32_t>(NPATCFGX_Bits::ATTWAIT), pccard_config_.attribute_timing->wt - 1,
               static_cast<uint32_t>(NPATCFGX_Bits::ATTHLD), pccard_config_.attribute_timing->ht,
               static_cast<uint32_t>(NPATCFGX_Bits::ATTHIZ), pccard_config_.attribute_timing->dbhzt - 1);

    write_bit_ranges(*this, EXMC_Regs::PIOTCFG3,
               static_cast<uint32_t>(PIOTCFG3_Bits::IOSET), pccard_config_.io_timing->st - 1,
               static_cast<uint32_t>(PIOTCFG3_Bits::IOWAIT), pccard_config_.io_timing->wt - 1,
               static_cast<uint32_t>(PIOTCFG3_Bits::IOHLD), pccard_config_.io_timing->ht,
               static_cast<uint32_t>(PIOTCFG3_Bits::IOHIZ), pccard_config_.io_timing->dbhzt - 1);
}

/**
 * @brief Enable or disable the PC Card block.
 *
 * This function sets the NDBKEN bit in the NPCTL3 register to enable or
 * disable the PC Card block. If enabling, this function should be called
 * after the pccard_init() function to ensure that the PC Card block is
 * properly configured.
 *
 * @param enable Set to true to enable the PC Card block, false to disable it.
 */
void EXMC::set_pccard_enable(bool enable) {
    write_bit(*this, EXMC_Regs::NPCTL3, static_cast<uint32_t>(NPCTLX_Bits::NDBKEN), enable);
}

/**
 * @brief Set the page size for a NOR/SRAM block.
 *
 * This function configures the page size for the specified NOR/SRAM block by
 * writing to the appropriate bits in the SNCTL register.
 *
 * @param block The block number for which to set the page size.
 * @param size The page size to be set, as a value from the Page_Size enum.
 */
void EXMC::set_nor_sram_page_size(Block_Number block, Page_Size size) {
    EXMC_Regs snctl_offset = get_snctl_offset(block);
    write_bit_range(*this, snctl_offset, static_cast<uint32_t>(SNCTLX_Bits::CPS), static_cast<uint32_t>(size));
}

/**
 * @brief Enable or disable ECC for a NAND Flash block.
 *
 * This function sets the ECCEN bit in the NPCTL register to enable or
 * disable the ECC for the specified NAND Flash block. If enabling,
 * this function should be called after the nand_init() function to
 * ensure that the NAND Flash block is properly configured.
 *
 * @param npc_block The NAND Flash block for which to set ECC enable.
 * @param enable Set to true to enable ECC, false to disable it.
 */
void EXMC::set_nand_ecc_enable(NPC_Block npc_block, bool enable) {
    EXMC_Regs npctl_offset = get_npctl_offset(npc_block);
    write_bit(*this, npctl_offset, static_cast<uint32_t>(NPCTLX_Bits::ECCEN), enable);
}

/**
 * @brief Read the ECC value from the specified NAND Flash block.
 *
 * This function returns the current ECC value stored in the NECC register
 * for the specified NAND Flash block.
 *
 * @param npc_block The NAND Flash block for which to read the ECC.
 * @returns The ECC value stored in the NECC register.
 */
uint32_t EXMC::get_ecc(NPC_Block npc_block) {
    EXMC_Regs necc_offset = get_necc_offset(npc_block);
    return read_register<uint32_t>(*this, necc_offset);
}

/**
 * @brief Retrieves the status of a specified flag for a given NAND Flash block.
 * 
 * This function reads the current status of a specified status flag 
 * from the interrupt enable register of the designated NAND Flash block.
 * 
 * @param npc_block The NAND Flash block from which to read the flag status.
 * @param flag The specific status flag to be checked.
 * @return True if the flag is set, otherwise false.
 */
bool EXMC::get_flag(NPC_Block npc_block, Status_Flags flag) {
    EXMC_Regs npinten_offset = get_npinten_offset(npc_block);
    return read_bit(*this, npinten_offset, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears a specified status flag for a given NAND Flash block.
 * 
 * This function writes a 0 to the corresponding bit of the status flag in 
 * the interrupt enable register, effectively clearing the flag for the 
 * specified NAND Flash block.
 * 
 * @param npc_block The NAND Flash block for which the flag should be cleared.
 * @param flag The specific status flag to clear.
 */
void EXMC::clear_flag(NPC_Block npc_block, Status_Flags flag) {
    EXMC_Regs npinten_offset = get_npinten_offset(npc_block);
    write_bit(*this, npinten_offset, static_cast<uint32_t>(flag), false);
}

/**
 * @brief Retrieves the status of a specified interrupt flag for a given NAND Flash block.
 * 
 * This function reads the current status of a specified interrupt flag from the interrupt enable register of the designated NAND Flash block.
 * The function will return true if the flag is set, and the interrupt is enabled for the flag, otherwise false.
 * 
 * @param npc_block The NAND Flash block from which to read the interrupt flag status.
 * @param flag The specific interrupt flag to be checked.
 * @return True if the flag is set and the interrupt is enabled, otherwise false.
 */
bool EXMC::get_interrupt_flag(NPC_Block npc_block, Interrupt_Flags flag) {
    EXMC_Regs npinten_offset = get_npinten_offset(npc_block);

    bool intr_status = false;
    bool flag_status = read_bit(*this, npinten_offset, static_cast<uint32_t>(flag));

    switch (flag) {
    case Interrupt_Flags::INTR_FLAG_RISING:
        intr_status = read_bit(*this, npinten_offset, static_cast<uint32_t>(Interrupt_Type::INTR_RISING_EN));
        break;
    case Interrupt_Flags::INTR_FLAG_LEVEL:
        intr_status = read_bit(*this, npinten_offset, static_cast<uint32_t>(Interrupt_Type::INTR_LEVEL_EN));
        break;
    case Interrupt_Flags::INTR_FLAG_FALLING:
        intr_status = read_bit(*this, npinten_offset, static_cast<uint32_t>(Interrupt_Type::INTR_FALLING_EN));
        break;
    default:
        break;
    }

    return (flag_status && intr_status);
}

/**
 * @brief Clears a specified interrupt flag for a given NAND Flash block.
 * 
 * This function disables the interrupt for the specified flag by clearing the
 * interrupt enable bit for the flag in the interrupt enable register of the
 * designated NAND Flash block.
 * 
 * @param npc_block The NAND Flash block for which to clear the interrupt flag.
 * @param flag The specific interrupt flag to be cleared.
 */
void EXMC::clear_interrupt_flag(NPC_Block npc_block, Interrupt_Flags flag) {
    EXMC_Regs npinten_offset = get_npinten_offset(npc_block);
    write_bit(*this, npinten_offset, static_cast<uint32_t>(flag), false);
}

/**
 * @brief Enables or disables an interrupt for a specified NAND Flash block.
 * 
 * This function enables or disables interrupts for the specified type
 * for the given NAND Flash block. The interrupts are enabled or disabled
 * by setting or clearing the interrupt enable bit for the respective type
 * in the interrupt enable register of the designated NAND Flash block.
 * 
 * @param npc_block The NAND Flash block for which to enable or disable the interrupt.
 * @param type The specific interrupt type to enable or disable.
 * @param enable True to enable the interrupt, false to disable it.
 */
void EXMC::set_interrupt_enable(NPC_Block npc_block, Interrupt_Type type, bool enable) {
    EXMC_Regs npinten_offset = get_npinten_offset(npc_block);
    write_bit(*this, npinten_offset, static_cast<uint32_t>(type), enable);
}


} // namespace exmc
