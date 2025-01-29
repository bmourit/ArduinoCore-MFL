//
// MFL gd32f30x SPI peripheral register access in C++
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

namespace spi {


///////////////////////////// BASE ADDRESS /////////////////////////////

enum class SPI_Base : uint32_t {
    SPI0_BASE,
    SPI1_BASE,
    SPI2_BASE,
    INVALID
};

static inline constexpr uintptr_t SPI_baseAddress[] = {
    0x40013000, // SPI0
    0x40003800, // SPI1
    0x40003C00  // SPI2
};


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class SPI_Regs : uint32_t {
    CTL0 = 0x00,
    CTL1 = 0x04,
    STAT = 0x08,
    DATA = 0x0C,
    CRCPOLY = 0x10,
    RCRC = 0x14,
    TCRC = 0x18,
    I2SCTL = 0x1C,
    I2SPSC = 0x20,
    QCTL = 0x80
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL0_Bits : uint32_t {
    CKPH = 0,
    CKPL = 1,
    MSTMOD = 2,
    PSC = REG_BIT_DEF(3, 5),
    SPIEN = 6,
    LF = 7,
    SWNSS = 8,
    SWNSSEN = 9,
    RO = 10,
    FF16 = 11,
    CRCNT = 12,
    CRCEN = 13,
    BDOEN = 14,
    BDEN = 15
};

enum class CTL1_Bits : uint8_t {
    DMAREN = 0,
    DMATEN = 1,
    NSSDRV = 2,
    NSSP = 3,
    TMOD = 4,
    ERRIE = 5,
    RBNEIE = 6,
    TBEIE = 7
};

enum class STAT_Bits : uint8_t {
    RBNE = 0,
    TBE = 1,
    I2SCH = 2,
    TXURERR = 3,
    CRCERR = 4,
    CONFERR = 5,
    RXORERR = 6,
    TRANS = 7,
    FERR = 8
};

enum class SPI_16Bit : uint32_t {
    MASK_16BIT = REG_BIT_DEF(0, 15)
};

enum class I2SCTL_Bits : uint32_t {
    CHLEN = 0,
    DTLEN = REG_BIT_DEF(1, 2),
    CKPL = 3,
    I2SSTD = REG_BIT_DEF(4, 5),
    PCMSMOD = 7,
    I2SOPMOD = REG_BIT_DEF(8, 9),
    I2SEN = 10,
    I2SSEL = 11
};

enum class I2SPSC_Bits : uint32_t {
    DIV = REG_BIT_DEF(0, 7),
    OF = 8,
    MCKOEN = 9
};

enum class QCTL_Bits : uint8_t {
    QMOD = 0,
    QRD = 1,
    IO23_DRV = 2
};

// Alias
enum class Status_Flags : uint8_t {
    FLAG_RBNE = 0,
    FLAG_TBE = 1,
    FLAG_I2SCH = 2,
    FLAG_TXURERR = 3,
    FLAG_CRCERR = 4,
    FLAG_CONFERR = 5,
    FLAG_RXORERR = 6,
    FLAG_TRANS = 7,
    FLAG_FERR = 8
};


///////////////////////////// ENUMS /////////////////////////////

enum class Mode_Select : uint8_t {
    MASTER,
    SLAVE
};

enum class Direction_Mode : uint8_t {
    BIDIRECTIONAL_RECEIVE,
    BIDIRECTIONAL_TRANSMIT
};

enum class Operational_Mode : uint8_t {
    MFD_MODE,	// Master Full-Duplex
    MTU_MODE,	// Master Transmission Unidirection
    MRU_MODE,	// Master Reception Unidirectional
    MTB_MODE,	// Master Transmission Bidirectional
    MRB_MODE,	// Master Reception Bidirectional
    SFD_MODE,	// Slave Full-Duplex
    STU_MODE,	// Slave Transmission Unidirection
    SRU_MODE,	// Slave Reception Unidirectional
    STB_MODE,	// Slave Transmission Bidirectional
    SRB_MODE	// SLAVE Reception Bidirectional
};

enum class Frame_Format : uint8_t {
    FF_8BIT,
    FF_16BIT
};

enum class NSS_Type : uint8_t {
    HARDWARE_NSS,
    SOFTWARE_NSS
};

enum class Endian_Type : uint8_t {
    MSBF,
    LSBF
};

enum class Clock_Polarity : uint8_t {
    PULL_LOW,
    PULL_HIGH
};

enum class Clock_Phase : uint8_t {
    PHASE_FIRST_EDGE,
    PHASE_SECOND_EDGE
};

enum class PCLK_Divider : uint8_t {
    PCLK_2,
    PCLK_4,
    PCLK_8,
    PCLK_16,
    PCLK_32,
    PCLK_64,
    PCLK_128,
    PCLK_256
};

enum class DMA_Direction : uint8_t {
    DMA_TX,
    DMA_RX
};

enum class CRC_Direction : uint8_t {
    CRC_TX,
    CRC_RX
};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_TBE,
    INTR_FLAG_RBNE,
    INTR_FLAG_RXORERR,
    INTR_FLAG_CONFERR,
    INTR_FLAG_CRCERR,
    INTR_FLAG_TXURERR,
    INTR_FLAG_FER
};

enum class Interrupt_Type : uint8_t {
    INTR_ERR = 5U,
    INTR_RBNE = 6U,
    INTR_TBE = 7U
};

enum class SPI_Error_Type : uint8_t {
    OK,
    INVALID_SPI,
    INVALID_OPERATION,
    INITIALIZATION_FAILED,
    INVALID_SELECTION
};


///////////////////////////// STRUCTURES /////////////////////////////

struct Interrupt_Flags_Config {
    SPI_Regs stat_reg;
    SPI_Regs ctl_reg;
    uint32_t stat_bit;
    uint32_t ctl_bit;
};

static inline constexpr std::array<Interrupt_Flags_Config, 7> interrupt_flags_config {{
    {SPI_Regs::STAT, SPI_Regs::CTL1, REG_BIT_DEF(1, 1), REG_BIT_DEF(7, 7)},
    {SPI_Regs::STAT, SPI_Regs::CTL1, REG_BIT_DEF(0, 0), REG_BIT_DEF(6, 6)},
    {SPI_Regs::STAT, SPI_Regs::CTL1, REG_BIT_DEF(6, 6), REG_BIT_DEF(5, 5)},
    {SPI_Regs::STAT, SPI_Regs::CTL1, REG_BIT_DEF(5, 5), REG_BIT_DEF(5, 5)},
    {SPI_Regs::STAT, SPI_Regs::CTL1, REG_BIT_DEF(4, 4), REG_BIT_DEF(5, 5)},
    {SPI_Regs::STAT, SPI_Regs::CTL1, REG_BIT_DEF(3, 3), REG_BIT_DEF(5, 5)},
    {SPI_Regs::STAT, SPI_Regs::CTL1, REG_BIT_DEF(8, 8), REG_BIT_DEF(5, 5)}
}};

struct SPI_Clock_Config {
    rcu::RCU_PCLK clock_reg;
    rcu::RCU_PCLK_Reset reset_reg;
};

static inline constexpr std::array<SPI_Clock_Config, 3> SPI_pclk_index {{
    {rcu::RCU_PCLK::PCLK_SPI0, rcu::RCU_PCLK_Reset::PCLK_SPI0RST},
    {rcu::RCU_PCLK::PCLK_SPI1, rcu::RCU_PCLK_Reset::PCLK_SPI1RST},
    {rcu::RCU_PCLK::PCLK_SPI2, rcu::RCU_PCLK_Reset::PCLK_SPI2RST}
}};

struct SPI_Config {
    Operational_Mode operational_mode;
    Frame_Format frame_format;
    NSS_Type nss_type;
    PCLK_Divider pclk_divider;
    Endian_Type msbf;
    Clock_Polarity polarity_pull;
    Clock_Phase clock_phase;
};


///////////////////////////// INITIALIZATION DEFAULTS /////////////////////////////

static inline const SPI_Config default_config = {
    Operational_Mode::SFD_MODE,
    Frame_Format::FF_8BIT,
    NSS_Type::HARDWARE_NSS,
    PCLK_Divider::PCLK_2,
    Endian_Type::MSBF,
    Clock_Polarity::PULL_LOW,
    Clock_Phase::PHASE_FIRST_EDGE,
};

} // namespace spi
