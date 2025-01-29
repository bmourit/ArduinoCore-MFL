//
// MFL gd32f30x USART peripheral register access in C++
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

namespace usart {


///////////////////////////// BASE ADDRESS /////////////////////////////

enum class USART_Base : uint32_t {
    USART0_BASE,
    USART1_BASE,
    USART2_BASE,
    UART3_BASE,
    UART4_BASE,
    INVALID
};

static inline constexpr uintptr_t USART_baseAddress[] = {
    0x40013800U,    // USART0
    0x40004400U,    // USART1
    0x40004800U,    // USART2
    0x40004C00U,    // UART3
    0x40005000U,    // UART4
    0x00000000U     // INVALID placeholder
};


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class USART_Regs : uint32_t {
    STAT0 = 0x00U,
    DATA = 0x04U,
    BAUD = 0x08U,
    CTL0 = 0x0CU,
    CTL1 = 0x10U,
    CTL2 = 0x14U,
    GP = 0x18U,
    CTL3 = 0x80U,
    RT = 0x84U,
    STAT1 = 0x88U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class STAT0_Bits : uint8_t {
    PERR = 0,
    FERR = 1,
    NERR = 2,
    ORERR = 3,
    IDLEF = 4,
    RBNE = 5,
    TC = 6,
    TBE = 7,
    LBDF = 8,
    CTSF = 9
};

enum class DATA_Bits : uint32_t {
    DATA = REG_BIT_DEF(0, 8)
};

enum class BAUD_Bits : uint32_t {
    FRADIV = REG_BIT_DEF(0, 3),
    INTDIV = REG_BIT_DEF(4, 15),
    BAUD = REG_BIT_DEF(0, 15)	// Simplifies writing baudrate
};

enum class CTL0_Bits : uint32_t {
    SBKCMD = 0,
    RWU = 1,
    REN = 2,
    TEN = 3,
    IDLEIE = 4,
    RBNEIE = 5,
    TCIE = 6,
    TBEIE = 7,
    PERRIE = 8,
    PM = 9,
    PCEN = 10,
    PMEN = REG_BIT_DEF(9, 10),	// Convenience
    WM = 11,
    WL = 12,
    UEN = 13
};

enum class CTL1_Bits : uint32_t {
    ADDR = REG_BIT_DEF(0, 3),
    LBLEN = 5,
    LBDIE = 6,
    CLEN = 8,
    CPH = 9,
    CPL = 10,
    CKEN = 11,
    STB = REG_BIT_DEF(12, 13),
    LMEN = 14
};

enum class CTL2_Bits : uint32_t {
    ERRIE = 0,
    IREN = 1,
    IRLP = 2,
    HDEN = 3,
    NKEN = 4,
    SCEN = 5,
    DENR = 6,
    DENT = 7,
    RTSEN = 8,
    CTSEN = 9,
    CTSIE = 10
};

enum class GP_Bits : uint32_t {
    PSC = REG_BIT_DEF(0, 7),
    GUAT = REG_BIT_DEF(8, 15)
};

enum class CTL3_Bits : uint32_t {
    RTEN = 0,
    SCRTNUM = REG_BIT_DEF(1, 3),
    RTIE = 4,
    EBIE = 5,
    RINV = 8,
    TINV = 9,
    DINV = 10,
    MSBF = 11
};

enum class RT_Bits : uint32_t {
    RT = REG_BIT_DEF(0, 23),
    BL = REG_BIT_DEF(24, 31)
};

enum class STAT1_Bits : uint32_t {
    RTF = 11,
    EBF = 12,
    BSY = 16
};


///////////////////////////// ENUMS /////////////////////////////

enum class Direction_Mode : uint8_t {
    RX_MODE,
    TX_MODE,
    RXTX_MODE,
    RXTX_OFF
};

enum class Parity_Mode : uint8_t {
    PM_NONE = 0,
    PM_EVEN = 2,
    PM_ODD = 3
};

enum class Word_Length : uint8_t {
    WL_8BITS,
    WL_9BITS
};

enum class Wakeup_Mode : uint8_t {
    WM_IDLE,
    WM_ADDR
};

enum class Break_Length : uint8_t {
    LENGTH_10B,
    LENGTH_11B
};

enum class Pulse_Length : uint8_t {
    EXT_PULSE_DISABLE,
    EXT_PULSE_ENABLE
};

enum class Clock_Phase : uint8_t {
    FIRST_CLOCK,
    SECOND_CLOCK
};

enum class Clock_Polarity : uint8_t {
    POLARITY_LOW,
    POLARITY_HIGH
};

enum class Stop_Bits : uint8_t {
    STB_1BIT,
    STB_0_5BIT,
    STB_2BIT,
    STB_1_5BIT
};

enum class MSBF_Mode : uint8_t {
    MSBF_LSB,
    MSBF_MSB
};

enum class Inversion_Method : uint8_t {
    DATA,
    TRANSMISSION,
    RECEPTION
};

enum class IrDA_Power : uint8_t {
    NORMAL,
    LOW
};

enum class Status_Flags : uint8_t {
    FLAG_PERR,	// STAT0, PERR
    FLAG_FERR,	// STAT0, FERR
    FLAG_NERR,	// STAT0, NERR
    FLAG_ORERR,	// STAT0, ORERR
    FLAG_IDLEF,	// STAT0, IDLEF
    FLAG_RBNE,	// STAT0, RBNE
    FLAG_TC,	// STAT0, TC
    FLAG_TBE,	// STAT0, TBE
    FLAG_LBDF,	// STAT0, LBDF
    FLAG_CTSF,	// STAT0, CTSF
    FLAG_RTF,	// STAT1, RTF
    FLAG_EBF,	// STAT1, EBF
    FLAG_BSY	// STAT1, BSY
};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_CTL0_PERR,    // STAT0, PERR CTL0, PERRIE
    INTR_FLAG_CTL0_TBE,		// STAT0, TBE CTL0, TBEIE
    INTR_FLAG_CTL0_TC,		// STAT0, TC CTL0, TCIE
    INTR_FLAG_CTL0_RBNE,    // STAT0, RBNE CTL0, RBNEIE
    INTR_FLAG_CTL0_ORERR,	// STAT0, ORERR CTL0, RBNEIE
    INTR_FLAG_CTL0_IDLEF,	// STAT0, IDLEF CTL0, IDLEIE
    INTR_FLAG_CTL1_LBDF,    // STAT0, LBDF CTL1, LBDIE
    INTR_FLAG_CTL2_CTSF,    // STAT0, CTSF CTL2, CTSIE
    INTR_FLAG_CTL2_ORERR,	// STAT0, ORERR CTL2, ERRIE
    INTR_FLAG_CTL2_NERR,    // STAT0, NERR CTL2, ERRIE
    INTR_FLAG_CTL2_FERR,    // STAT0, FERR CTL2, ERRIE
    INTR_FLAG_CTL3_EBF,		// STAT1, EBF CTL3, EBIE
    INTR_FLAG_CTL3_RTF		// STAT1, RTF CTL3, RTIE
};

enum class Interrupt_Type : uint8_t {
    INTR_PERRIE,	// CTL0
    INTR_TBEIE,		// CTL0
    INTR_TCIE,		// CTL0
    INTR_RBNEIE,	// CTL0
    INTR_IDLEIE,	// CTL0
    INTR_LBDIE,		// CTL1
    INTR_CTSIE,		// CTL2
    INTR_ERRIE,		// CTL2
    INTR_EBIE,		// CTL3
    INTR_RTIE,		// CTL3
    INTR_COUNT      // End marker for callbacks
};

enum class Inversion_Type : uint8_t {
    DINV_ENABLE,
    DINV_DISABLE,
    TXPIN_ENABLE,
    TXPIN_DISABLE,
    RXPIN_ENABLE,
    RXPIN_DISABLE
};

enum class USART_DMA_Config : uint8_t {
    DMA_NONE,
    DMA_TX,
    DMA_RX,
    DMA_DUAL
};

enum class USART_State : uint8_t {
    IDLE,
    BUSY_RECEIVE,
    BUSY_TRANSMIT,
    BUSY_COMPLETION,
    ERROR
};

enum class USART_Error_Type : uint8_t {
    OK,
    INVALID_USART,
    INVALID_OPERATION,
    INITIALIZATION_FAILED,
    INVALID_SELECTION,
    USART_INSTANCE_FAILED,
    PARITY_ERROR,
    NOISE_ERROR,
    FRAME_ERROR,
    DMA_TRANSFER_ERRROR,
    TIMEOUT
};


///////////////////////////// STRUCTURES /////////////////////////////

struct USART_Clock_Config {
    rcu::RCU_PCLK clock_reg;
    rcu::RCU_PCLK_Reset reset_reg;
};

static inline constexpr std::array<USART_Clock_Config, 5> USART_pclk_index {{
    {rcu::RCU_PCLK::PCLK_USART0, rcu::RCU_PCLK_Reset::PCLK_USART0RST},
    {rcu::RCU_PCLK::PCLK_USART1, rcu::RCU_PCLK_Reset::PCLK_USART1RST},
    {rcu::RCU_PCLK::PCLK_USART2, rcu::RCU_PCLK_Reset::PCLK_USART2RST},
    {rcu::RCU_PCLK::PCLK_UART3, rcu::RCU_PCLK_Reset::PCLK_UART3RST},
    {rcu::RCU_PCLK::PCLK_UART4, rcu::RCU_PCLK_Reset::PCLK_UART4RST}
}};

struct Status_Config {
    USART_Regs reg;
    uint32_t bit;
};

static inline constexpr std::array<Status_Config, 13> status_config {{
    {USART_Regs::STAT0, REG_BIT_DEF(0, 0)},
    {USART_Regs::STAT0, REG_BIT_DEF(1, 1)},
    {USART_Regs::STAT0, REG_BIT_DEF(2, 2)},
    {USART_Regs::STAT0, REG_BIT_DEF(3, 3)},
    {USART_Regs::STAT0, REG_BIT_DEF(4, 4)},
    {USART_Regs::STAT0, REG_BIT_DEF(5, 5)},
    {USART_Regs::STAT0, REG_BIT_DEF(6, 6)},
    {USART_Regs::STAT0, REG_BIT_DEF(7, 7)},
    {USART_Regs::STAT0, REG_BIT_DEF(8, 8)},
    {USART_Regs::STAT0, REG_BIT_DEF(9, 9)},
    {USART_Regs::STAT1, REG_BIT_DEF(11, 11)},
    {USART_Regs::STAT1, REG_BIT_DEF(12, 12)},
    {USART_Regs::STAT1, REG_BIT_DEF(16, 16)}
}};

struct Interrupt_Flags_Config {
    USART_Regs reg;
    USART_Regs enable_reg;
    bool invert;
    uint32_t bit;
    uint32_t enable_bit;
};

static inline constexpr std::array<Interrupt_Flags_Config, 13> interrupt_flags_config {{
    {USART_Regs::STAT0, USART_Regs::CTL0, false, REG_BIT_DEF(0, 0), REG_BIT_DEF(8, 8)},
    {USART_Regs::STAT0, USART_Regs::CTL0, false, REG_BIT_DEF(7, 7), REG_BIT_DEF(7, 7)},
    {USART_Regs::STAT0, USART_Regs::CTL0, false, REG_BIT_DEF(6, 6), REG_BIT_DEF(6, 6)},
    {USART_Regs::STAT0, USART_Regs::CTL0, false, REG_BIT_DEF(5, 5), REG_BIT_DEF(5, 5)},
    {USART_Regs::STAT0, USART_Regs::CTL0, false, REG_BIT_DEF(3, 3), REG_BIT_DEF(5, 5)},
    {USART_Regs::STAT0, USART_Regs::CTL0, false, REG_BIT_DEF(4, 4), REG_BIT_DEF(4, 4)},
    {USART_Regs::STAT0, USART_Regs::CTL1, false, REG_BIT_DEF(8, 8), REG_BIT_DEF(6, 6)},
    {USART_Regs::STAT0, USART_Regs::CTL2, false, REG_BIT_DEF(9, 9), REG_BIT_DEF(10, 10)},
    {USART_Regs::STAT0, USART_Regs::CTL2, false, REG_BIT_DEF(3, 3), REG_BIT_DEF(0, 0)},
    {USART_Regs::STAT0, USART_Regs::CTL2, false, REG_BIT_DEF(2, 2), REG_BIT_DEF(0, 0)},
    {USART_Regs::STAT0, USART_Regs::CTL2, false, REG_BIT_DEF(1, 1), REG_BIT_DEF(0, 0)},
    {USART_Regs::STAT1, USART_Regs::CTL3, true, REG_BIT_DEF(12, 12), REG_BIT_DEF(5, 5)},
    {USART_Regs::STAT1, USART_Regs::CTL3, true, REG_BIT_DEF(11, 11), REG_BIT_DEF(4, 4)}
}};

struct Interrupt_Config {
    USART_Regs reg;
    bool invert;
    uint32_t bit_info;
};

static inline constexpr std::array<Interrupt_Config, 10> interrupt_config {{
    {USART_Regs::CTL0, false, REG_BIT_DEF(8, 8)},
    {USART_Regs::CTL0, false, REG_BIT_DEF(7, 7)}, 
    {USART_Regs::CTL0, false, REG_BIT_DEF(6, 6)},
    {USART_Regs::CTL0, false, REG_BIT_DEF(5, 5)},
    {USART_Regs::CTL0, false, REG_BIT_DEF(4, 4)},
    {USART_Regs::CTL1, false, REG_BIT_DEF(6, 6)},
    {USART_Regs::CTL2, false, REG_BIT_DEF(10, 10)},
    {USART_Regs::CTL2, false, REG_BIT_DEF(0, 0)},
    {USART_Regs::CTL3, true, REG_BIT_DEF(5, 5)},
    {USART_Regs::CTL3, true, REG_BIT_DEF(4, 4)}
}};

struct USART_sc_params {
    uint8_t guat;
    uint8_t sc_auto_retry;
    uint8_t sc_block_length;
};

struct USART_sync_params {
    uint32_t clen;
    uint32_t cph;
    uint32_t cpl;
};

struct USART_irda_params {
    uint32_t power_level;
    uint8_t prescaler;
};

struct USART_Config {
    uint32_t baudrate;          // Must never be 0
    USART_DMA_Config dma_ops;   // DMA pin usage flag: off, RX pin, TX pin, or both pins
    Parity_Mode parity;
    Word_Length word_length;
    Stop_Bits stop_bits;
    Direction_Mode direction;
    USART_State state;
    uint8_t interrupt_prepriority;
    uint8_t interrupt_subpriority;
    uint8_t last_data;
};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint8_t defaultIrqPreemptPriority = 1U;
static inline constexpr uint8_t defaultIrqSubPriority = 0U;

// Allow user to set buffer sizes
#ifdef USART_RX_BUFFER_SIZE
static inline constexpr uint32_t RxBufferSize = USART_RX_BUFFER_SIZE;
#else
static inline constexpr uint32_t RxBufferSize = 128;
#endif

#ifdef USART_TX_BUFFER_SIZE
static inline constexpr uint32_t TxBufferSize = USART_TX_BUFFER_SIZE;
#else
static inline constexpr uint32_t TxBufferSize = 64;
#endif


///////////////////////////// INITIALIZATION DEFAULTS /////////////////////////////

static inline const USART_Config default_config = {
    115200U,
    USART_DMA_Config::DMA_NONE,
    Parity_Mode::PM_NONE,
    Word_Length::WL_8BITS,
    Stop_Bits::STB_1BIT,
    Direction_Mode::RXTX_MODE,
    USART_State::IDLE,
    defaultIrqPreemptPriority,
    defaultIrqSubPriority,
    0U,
};

} // namespace usart
