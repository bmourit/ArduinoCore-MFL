//
// MFL gd32f30x I2C peripheral register access in C++
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
#include "rcu_config.hpp"

namespace i2c {


///////////////////////////// BASE ADDRESS /////////////////////////////

enum class I2C_Base : uint32_t {
    I2C0_BASE,
    I2C1_BASE,
    INVALID
};

static inline constexpr uintptr_t I2C_baseAddress[] = {
    0x40005400U, // I2C0
    0x40005800U  // I2C1
};


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class I2C_Regs : uint32_t {
    CTL0 = 0x00U,
    CTL1 = 0x04U,
    SADDR0 = 0x08U,
    SADDR1 = 0x0CU,
    DATA = 0x10U,
    STAT0 = 0x14U,
    STAT1 = 0x18U,
    CKCFG = 0x1CU,
    RT = 0x20U,
    FMPCFG = 0x90U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL0_Bits : uint32_t {
    I2CEN = 0,
    SMBEN = 1,
    SMBSEL = 2,
    ARPEN = REG_BIT_DEF(3, 5),
    PECEN = 6,
    GCEN = 7,
    SS = 8,
    START = 9,
    STOP = 10,
    ACKEN = 11,
    POAP = 12,
    PECTRANS = 13,
    SALT = 14,
    SRESET = 15
};

enum class CTL1_Bits : uint32_t {
    I2CCLK = REG_BIT_DEF(0, 6),
    ERRIE = 8,
    EVIE = 9,
    BUFIE = 10,
    DMAON = 11,
    DMALST = 12
};

enum class SADDR0_Bits : uint32_t {
    ADDRESS_10BIT = REG_BIT_DEF(0, 9),
    ADDRESS_7BIT = REG_BIT_DEF(1, 7),
    ADDFORMAT = 15
};

enum class SADDR1_Bits : uint32_t {
    DUADEN = 0,
    ADDRESS2 = REG_BIT_DEF(1, 7)
};

enum class DATA_Bits : uint32_t {
    TRB = REG_BIT_DEF(0, 7)
};

enum class STAT0_Bits : uint32_t {
    SBSEND = 0,
    ADDSEND = 1,
    BTC = 2,
    ADD10SEND = 3,
    STPDET = 4,
    RBNE = 6,
    TBE = 7,
    BERR = 8,
    LOSTARB = 9,
    AERR = 10,
    OUERR = 11,
    PECERR = 12,
    SMBTO = 14,
    SMBALT = 15
};

enum class STAT1_Bits : uint32_t {
    MASTER = 0,
    I2CBSY = 1,
    TR = 2,
    RXGC = 4,
    DEFSMB = 5,
    HSTSMB = 6,
    DUMODF = 7,
    PECV = REG_BIT_DEF(8, 15)
};

enum class CKCFG_Bits : uint32_t {
    CLKC = REG_BIT_DEF(0, 11),
    DTCY = 14,
    FAST = 15
};

enum class RT_Bits : uint32_t {
    RISETIME = REG_BIT_DEF(0, 6)
};

enum class FMPCFG_Bits : uint32_t {
    FMPEN = 0
};


///////////////////////////// ENUMS /////////////////////////////

enum class Status_Flags : uint8_t {
    FLAG_SBSEND,
    FLAG_ADDSEND,
    FLAG_BTC,
    FLAG_ADD10SEND,
    FLAG_STPDET,
    FLAG_RBNE,
    FLAG_TBE,
    FLAG_BERR,
    FLAG_LOSTARB,
    FLAG_AERR,
    FLAG_OUERR,
    FLAG_PECERR,
    FLAG_SMBTO,
    FLAG_SMBALT,
    FLAG_MASTER,
    FLAG_I2CBSY,
    FLAG_TRS,
    FLAG_RXGC,
    FLAG_DEFSMB,
    FLAG_HSTSMB,
    FLAG_DUMOD
};

struct index_to_bits {
    I2C_Regs register_offset;
    uint32_t bit_info;
};

static inline constexpr std::array<index_to_bits, 21> status_flag_index {{
        {I2C_Regs::STAT0, REG_BIT_DEF(0, 0)},
        {I2C_Regs::STAT0, REG_BIT_DEF(1, 1)},
        {I2C_Regs::STAT0, REG_BIT_DEF(2, 2)},
        {I2C_Regs::STAT0, REG_BIT_DEF(3, 3)},
        {I2C_Regs::STAT0, REG_BIT_DEF(4, 4)},
        {I2C_Regs::STAT0, REG_BIT_DEF(6, 6)},
        {I2C_Regs::STAT0, REG_BIT_DEF(7, 7)},
        {I2C_Regs::STAT0, REG_BIT_DEF(8, 8)},
        {I2C_Regs::STAT0, REG_BIT_DEF(9, 9)},
        {I2C_Regs::STAT0, REG_BIT_DEF(10, 10)},
        {I2C_Regs::STAT0, REG_BIT_DEF(11, 11)},
        {I2C_Regs::STAT0, REG_BIT_DEF(12, 12)},
        {I2C_Regs::STAT0, REG_BIT_DEF(14, 14)},
        {I2C_Regs::STAT0, REG_BIT_DEF(15, 15)},
        {I2C_Regs::STAT1, REG_BIT_DEF(0, 0)},
        {I2C_Regs::STAT1, REG_BIT_DEF(1, 1)},
        {I2C_Regs::STAT1, REG_BIT_DEF(2, 2)},
        {I2C_Regs::STAT1, REG_BIT_DEF(4, 4)},
        {I2C_Regs::STAT1, REG_BIT_DEF(5, 5)},
        {I2C_Regs::STAT1, REG_BIT_DEF(6, 6)},
        {I2C_Regs::STAT1, REG_BIT_DEF(7, 7)}
    }};

enum class Clear_Flags : uint8_t {
    FLAG_SBSEND,
    FLAG_ADDSEND,
    FLAG_BERR,
    FLAG_LOSTARB,
    FLAG_AERR,
    FLAG_OUERR,
    FLAG_PECERR,
    FLAG_SMBTO,
    FLAG_SMBALT
};

static inline constexpr std::array<index_to_bits, 9> clear_flag_index {{
        {I2C_Regs::STAT0, REG_BIT_DEF(0, 0)},
        {I2C_Regs::STAT0, REG_BIT_DEF(1, 1)},
        {I2C_Regs::STAT0, REG_BIT_DEF(8, 8)},
        {I2C_Regs::STAT0, REG_BIT_DEF(9, 9)},
        {I2C_Regs::STAT0, REG_BIT_DEF(10, 10)},
        {I2C_Regs::STAT0, REG_BIT_DEF(11, 11)},
        {I2C_Regs::STAT0, REG_BIT_DEF(12, 12)},
        {I2C_Regs::STAT0, REG_BIT_DEF(14, 14)},
        {I2C_Regs::STAT0, REG_BIT_DEF(15, 15)}
    }};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_SBSEND,
    INTR_FLAG_ADDSEND,
    INTR_FLAG_BTC,
    INTR_FLAG_ADD10SEND,
    INTR_FLAG_STPDET,
    INTR_FLAG_RBNE,
    INTR_FLAG_TBE,
    INTR_FLAG_BERR,
    INTR_FLAG_LOSTARB,
    INTR_FLAG_AERR,
    INTR_FLAG_OUERR,
    INTR_FLAG_PECERR,
    INTR_FLAG_SMBTO,
    INTR_FLAG_SMBALT
};

struct double_index_to_bits {
    I2C_Regs register0_offset;
    uint32_t bit_info0;
    I2C_Regs register1_offset;
    uint32_t bit_info1;
};

static inline constexpr std::array<double_index_to_bits, 14> interrupt_flag_index {{
        {I2C_Regs::CTL1, REG_BIT_DEF(9, 9), I2C_Regs::STAT0, REG_BIT_DEF(0, 0)},
        {I2C_Regs::CTL1, REG_BIT_DEF(9, 9), I2C_Regs::STAT0, REG_BIT_DEF(1, 1)},
        {I2C_Regs::CTL1, REG_BIT_DEF(9, 9), I2C_Regs::STAT0, REG_BIT_DEF(2, 2)},
        {I2C_Regs::CTL1, REG_BIT_DEF(9, 9), I2C_Regs::STAT0, REG_BIT_DEF(3, 3)},
        {I2C_Regs::CTL1, REG_BIT_DEF(9, 9), I2C_Regs::STAT0, REG_BIT_DEF(4, 4)},
        {I2C_Regs::CTL1, REG_BIT_DEF(9, 9), I2C_Regs::STAT0, REG_BIT_DEF(6, 6)},
        {I2C_Regs::CTL1, REG_BIT_DEF(9, 9), I2C_Regs::STAT0, REG_BIT_DEF(7, 7)},
        {I2C_Regs::CTL1, REG_BIT_DEF(8, 8), I2C_Regs::STAT0, REG_BIT_DEF(8, 8)},
        {I2C_Regs::CTL1, REG_BIT_DEF(8, 8), I2C_Regs::STAT0, REG_BIT_DEF(9, 9)},
        {I2C_Regs::CTL1, REG_BIT_DEF(8, 8), I2C_Regs::STAT0, REG_BIT_DEF(10, 10)},
        {I2C_Regs::CTL1, REG_BIT_DEF(8, 8), I2C_Regs::STAT0, REG_BIT_DEF(11, 11)},
        {I2C_Regs::CTL1, REG_BIT_DEF(8, 8), I2C_Regs::STAT0, REG_BIT_DEF(12, 12)},
        {I2C_Regs::CTL1, REG_BIT_DEF(8, 8), I2C_Regs::STAT0, REG_BIT_DEF(14, 14)},
        {I2C_Regs::CTL1, REG_BIT_DEF(8, 8), I2C_Regs::STAT0, REG_BIT_DEF(15, 15)}
    }};

enum class Interrupt_Type : uint8_t {
    INTR_ERR,
    INTR_EV,
    INTR_BUF
};

static inline constexpr std::array<index_to_bits, 3> interrupt_type_index {{
        {I2C_Regs::CTL1, REG_BIT_DEF(8, 8)},
        {I2C_Regs::CTL1, REG_BIT_DEF(9, 9)},
        {I2C_Regs::CTL1, REG_BIT_DEF(10, 10)}
    }};

enum class Bus_Mode : uint8_t {
    I2C,
    SMBUS
};

enum class Bus_Type : uint8_t {
    DEVICE,
    HOST
};

enum class Transfer_Direction : uint8_t {
    RECEIVE = 1,
    TRANSMIT = 2
};

enum class ACK_Select : uint8_t {
    CURRENT,
    NEXT
};

enum class Stretch_Low : uint8_t {
    SCLSTRETCH_ENABLE,
    SCLSTRETCH_DISABLE
};

enum class Duty_Cycle : uint8_t {
    DTCY_2,
    DTCY_16_9
};

enum class Address_Format : uint8_t {
    FORMAT_7BITS,
    FORMAT_10BITS
};

enum class I2C_Error_Type : uint8_t {
    OK,
    BUSY,
    TIMEOUT,
    NACK_ADDRESS,
    NACK_DATA,
    DATA_SIZE_ERROR,
    INVALID_I2C,
    INVALID_OPERATION,
    INITIALIZATION_FAILED,
    INVALID_SELECTION,
    INVALID_CLOCK_FREQUENCY
};


///////////////////////////// STRUCTURES /////////////////////////////

struct I2C_Clock_Config {
    rcu::RCU_PCLK clock_reg;
    rcu::RCU_PCLK_Reset reset_reg;
};

static inline constexpr std::array<I2C_Clock_Config, 2> I2C_pclk_index {{
        {rcu::RCU_PCLK::PCLK_I2C0, rcu::RCU_PCLK_Reset::PCLK_I2C0RST},
        {rcu::RCU_PCLK::PCLK_I2C1, rcu::RCU_PCLK_Reset::PCLK_I2C1RST}
    }};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t MaximumClockSpeed = 60U;
static inline constexpr uint32_t MinimumClockSpeed = 2U;
static inline constexpr uint32_t AddressMask = 0x000003FFU;
static inline constexpr uint32_t Address2Mask = 0x000000FEU;


} // namespace i2c
