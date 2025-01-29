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

#include <stdlib.h>
#include <stdint.h>
#include <array>

#include "CONFIG.hpp"

namespace fmc {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class FMC_Regs : uint32_t {
    WS = 0x00U,
    KEY0 = 0x04U,
    OBKEY = 0x08U,
    STAT0 = 0x0CU,
    CTL0 = 0x10U,
    ADDR0 = 0x14U,
    OBSTAT = 0x1CU,
    WP = 0x20U,
    KEY1 = 0x44U,
    STAT1 = 0x4CU,
    CTL1 = 0x50U,
    ADDR1 = 0x54U,
    WSEN = 0xFCU,
    PID = 0x100U
};

enum class OB_Regs : uint32_t {
    SPC = 0x00U,
    USER = 0x02U,
    DATA1 = 0x04U,
    DATA2 = 0x06U,
    WP0 = 0x08U,
    WP1 = 0x0AU,
    WP2 = 0x0CU,
    WP3 = 0x0EU
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class WS_Bits : uint32_t {
    WSCNT = REG_BIT_DEF(0, 2)
};

enum class STAT0_Bits : uint8_t {
    BUSY = 0,
    PGERR = 2,
    WPERR = 4,
    ENDF = 5
};

enum class CTL0_Bits : uint8_t {
    PG = 0,
    PER = 1,
    MER = 2,
    OBPG = 4,
    OBER = 5,
    START = 6,
    LK = 7,
    OBWEN = 9,
    ERRIE = 10,
    ENDIE = 12
};

enum class OBSTAT_Bits : uint32_t {
    OBERR = 0,
    SPC = 1,
    USER = REG_BIT_DEF(2, 9),
    DATA = REG_BIT_DEF(10, 25)
};

enum class STAT1_Bits : uint8_t {
    BUSY = 0,
    PGERR = 2,
    WPERR = 4,
    ENDF = 5
};

enum class CTL1_Bits : uint8_t {
    PG = 0,
    PER = 1,
    MER = 2,
    START = 6,
    LK = 7,
    ERRIE = 10,
    ENDIE = 12
};

enum class WSEN_Bits : uint8_t {
    WSEN = 0,
    BPEN = 1
};

enum class SPC_Bits : uint32_t {
    SPC = REG_BIT_DEF(0, 7),
    SPC_N = REG_BIT_DEF(8, 15)
};

enum class USER_Bits : uint32_t {
    USER = REG_BIT_DEF(16, 23),
    USER_N = REG_BIT_DEF(24, 31)
};

enum class WPX_Bits : uint32_t {
    WP0 = REG_BIT_DEF(0, 7),
    WP1 = REG_BIT_DEF(8, 15),
    WP2 = REG_BIT_DEF(16, 23),
    WP3 = REG_BIT_DEF(24, 31)
};


///////////////////////////// ENUMS /////////////////////////////

struct index_to_bits {
    FMC_Regs register_offset;
    uint32_t bit_info;
};

enum class Status_Flags : uint8_t {
    FLAG_BANK0_BUSY,
    FLAG_BANK0_PGERR,
    FLAG_BANK0_WPERR,
    FLAG_BANK0_END,
    FLAG_OBERR,
    FLAG_BANK1_BUSY,
    FLAG_BANK1_PGERR,
    FLAG_BANK1_WPERR,
    FLAG_BANK1_END
};

static inline constexpr std::array<index_to_bits, 9> status_flag_index {{
    {FMC_Regs::STAT0, REG_BIT_DEF(0, 0)},
    {FMC_Regs::STAT0, REG_BIT_DEF(2, 2)},
    {FMC_Regs::STAT0, REG_BIT_DEF(4, 4)},
    {FMC_Regs::STAT0, REG_BIT_DEF(5, 5)},
    {FMC_Regs::OBSTAT, REG_BIT_DEF(0, 0)},
    {FMC_Regs::STAT1, REG_BIT_DEF(0, 0)},
    {FMC_Regs::STAT1, REG_BIT_DEF(2, 2)},
    {FMC_Regs::STAT1, REG_BIT_DEF(4, 4)},
    {FMC_Regs::STAT1, REG_BIT_DEF(5, 5)}
}};

struct index_to_reg_bits {
    FMC_Regs flag_register_offset;
    uint32_t flag_bit_info;
    FMC_Regs interrupt_register_offset;
    uint32_t interrupt_bit_info;
};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_BANK0_PGERR,
    INTR_FLAG_BANK0_WPERR,
    INTR_FLAG_BANK0_END,
    INTR_FLAG_BANK1_PGERR,
    INTR_FLAG_BANK1_WPERR,
    INTR_FLAG_BANK1_END
};

static inline constexpr std::array<index_to_reg_bits, 6> interrupt_flag_index {{
    {FMC_Regs::STAT0, REG_BIT_DEF(2, 2), FMC_Regs::CTL0, REG_BIT_DEF(10, 10)},
    {FMC_Regs::STAT0, REG_BIT_DEF(4, 4), FMC_Regs::CTL0, REG_BIT_DEF(10, 10)},
    {FMC_Regs::STAT0, REG_BIT_DEF(5, 5), FMC_Regs::CTL0, REG_BIT_DEF(12, 12)},
    {FMC_Regs::STAT1, REG_BIT_DEF(2, 2), FMC_Regs::CTL1, REG_BIT_DEF(10, 10)},
    {FMC_Regs::STAT1, REG_BIT_DEF(4, 4), FMC_Regs::CTL1, REG_BIT_DEF(10, 10)},
    {FMC_Regs::STAT1, REG_BIT_DEF(5, 5), FMC_Regs::CTL1, REG_BIT_DEF(12, 12)}
}};

enum class Interrupt_Types : uint8_t {
    INTR_BANK0_END,
    INTR_BANK0_ERR,
    INTR_BANK1_END,
    INTR_BANK1_ERR
};

static inline constexpr std::array<index_to_bits, 4> interrupt_type_index {{
    {FMC_Regs::CTL0, REG_BIT_DEF(12, 12)},
    {FMC_Regs::CTL0, REG_BIT_DEF(10, 10)},
    {FMC_Regs::CTL1, REG_BIT_DEF(12, 12)},
    {FMC_Regs::CTL1, REG_BIT_DEF(10, 10)}
}};

enum class Wait_State : uint8_t {
    WS_WSCNT_0,
    WS_WSCNT_1,
    WS_WSCNT_2
};

enum class OB_Watchdog_Type : uint8_t {
    FWDGT_HW,
    FWDGT_SW
};

enum class OB_Deep_Sleep : uint8_t {
    DEEPSLEEP_RESET = 0,
    DEEPSLEEP_NO_RESET = 2
};

enum class OB_Standby : uint8_t {
    STABDBY_RESET = 0,
    STANDBY_NO_RESET = 4
};

enum class OB_Boot_Bank : uint8_t {
    BOOT_BANK1 = 0,
    BOOT_BANK0 = 8
};

enum class OB_User_Mask : uint8_t {
    USER_MASK = 0xF0U
};

enum class OB_Security_Type : uint8_t {
    OB_NSPC = 0xA5U,		// No protection
    OB_USPC = 0xBBU 		// Under protection
};

enum class WP_Sector : uint8_t {
    OB_WP_0,
    OB_WP_1,
    OB_WP_2,
    OB_WP_3,
    OB_WP_4,
    OB_WP_5,
    OB_WP_6,
    OB_WP_7,
    OB_WP_8,
    OB_WP_9,
    OB_WP_10,
    OB_WP_11,
    OB_WP_12,
    OB_WP_13,
    OB_WP_14,
    OB_WP_15,
    OB_WP_16,
    OB_WP_17,
    OB_WP_18,
    OB_WP_19,
    OB_WP_20,
    OB_WP_21,
    OB_WP_22,
    OB_WP_23,
    OB_WP_24,
    OB_WP_25,
    OB_WP_26,
    OB_WP_27,
    OB_WP_28,
    OB_WP_29,
    OB_WP_30,
    OB_WP_31
};

enum class FMC_Error_Type : uint8_t {
    READY,
    BUSY,
    PG_ERROR,
    WP_ERROR,
    TIMEOUT
};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t Unlock_Key0 = 0x45670123U;
static inline constexpr uint32_t Unlock_Key1 = 0xCDEF89ABU;
static inline constexpr uint32_t Timeout_Count = 0x0FFF0000U;

static inline constexpr uint32_t Bank0_End_Address = 0x0807FFFFU;
static inline constexpr uint32_t Bank0_Size = 0x00000200U;
static inline constexpr uintptr_t Flash_Size_Addess = 0x1FFFF7E0U;


} // namespace fmc
