//
// MFL gd32f30x DMA peripheral register access in C++
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

namespace dma {


///////////////////////////// BASE ADDRESS /////////////////////////////

enum class DMA_Base : uint32_t {
    DMA0_BASE,
    DMA1_BASE,
    INVALID
};

static inline constexpr uintptr_t DMA_baseAddress[] = {
    0x40020000U,
    0x40020400U
};


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class DMA_Regs : uint32_t {
    INTF = 0x00U,
    INTC = 0x04U,
    // CHXCTL
    CH0CTL = 0x08U,
    CH1CTL = 0x1CU,
    CH2CTL = 0x30U,
    CH3CTL = 0x44U,
    CH4CTL = 0x58U,
    CH5CTL = 0x6CU,
    CH6CTL = 0x80U,
    // CHXCNT
    CH0CNT = 0x0CU,
    CH1CNT = 0x20U,
    CH2CNT = 0x34U,
    CH3CNT = 0x48U,
    CH4CNT = 0x5CU,
    CH5CNT = 0x70U,
    CH6CNT = 0x84U,
    // CHXPADDR
    CH0PADDR = 0x10U,
    CH1PADDR = 0x24U,
    CH2PADDR = 0x38U,
    CH3PADDR = 0x4CU,
    CH4PADDR = 0x60U,
    CH5PADDR = 0x74U,
    CH6PADDR = 0x88U,
    // CHXMADDR
    CH0MADDR = 0x14U,
    CH1MADDR = 0x28U,
    CH2MADDR = 0x3CU,
    CH3MADDR = 0x50U,
    CH4MADDR = 0x64U,
    CH5MADDR = 0x78U,
    CH6MADDR = 0x8CU,
    // INVALID
    INVALID = 0x90U
};

enum class Channel_Regs : uint32_t {
    CHXCTL,
    CHXCNT,
    CHXPADDR,
    CHXMADDR,
    INVALID
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class INTF_Bits : uint8_t {
    GIF0 = 0,
    FTFIF0 = 1,
    HTFIF0 = 2,
    ERRIF0 = 3,
    GIF1 = 4,
    FTFIF1 = 5,
    HTFIF1 = 6,
    ERRIF1 = 7,
    GIF2 = 8,
    FTFIF2 = 9,
    HTFIF2 = 10,
    ERRIF2 = 11,
    GIF3 = 12,
    FTFIF3 = 13,
    HTFIF3 = 14,
    ERRIF3 = 15,
    GIF4 = 16,
    FTFIF4 = 17,
    HTFIF4 = 18,
    ERRIF4 = 19,
    GIF5 = 20,
    FTFIF5 = 21,
    HTFIF5 = 22,
    ERRIF5 = 23,
    GIF6 = 24,
    FTFIF6 = 25,
    HTFIF6 = 26,
    ERRIF6 = 27,
    INVALID = 30
};

enum class INTC_Bits : uint8_t {
    GIFC0 = 0,
    FTFIFC0 = 1,
    HTFIFC0 = 2,
    ERRIFC0 = 3,
    GIFC1 = 4,
    FTFIFC1 = 5,
    HTFIFC1 = 6,
    ERRIFC1 = 7,
    GIFC2 = 8,
    FTFIFC2 = 9,
    HTFIFC2 = 10,
    ERRIFC2 = 11,
    GIFC3 = 12,
    FTFIFC3 = 13,
    HTFIFC3 = 14,
    ERRIFC3 = 15,
    GIFC4 = 16,
    FTFIFC4 = 17,
    HTFIFC4 = 18,
    ERRIFC4 = 19,
    GIFC5 = 20,
    FTFIFC5 = 21,
    HTFIFC5 = 22,
    ERRIFC5 = 23,
    GIFC6 = 24,
    FTFIFC6 = 25,
    HTFIFC6 = 26,
    ERRIFC6 = 27
};

enum class CHXCTL_Bits : uint32_t {
    CHEN = 0,
    FTFIE = 1,
    HTFIE = 2,
    ERRIE = 3,
    DIR = 4,
    CMEN = 5,
    PNAGA = 6,
    MNAGA = 7,
    PWIDTH = REG_BIT_DEF(8, 9),
    MWIDTH = REG_BIT_DEF(10, 11),
    PRIO = REG_BIT_DEF(12, 13),
    M2M = 14,
    ALL = REG_BIT_DEF(0, 14)
};


///////////////////////////// ENUMS /////////////////////////////

enum class DMA_Channel : uint8_t {
    CHANNEL0,
    CHANNEL1,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    INVALID
};

enum class Status_Flags : uint8_t {
    FLAG_GIF,
    FLAG_FTFIF,     // Full transfer finished interrupt
    FLAG_HTFIF,     // Half transfer finished interrupt
    FLAG_ERRIF,
    INVALID
};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_GIF,
    INTR_FLAG_FTFIF,
    INTR_FLAG_HTFIF,
    INTR_FLAG_ERRIF
};

enum class Interrupt_Type : uint8_t {
    INTR_FTFIE = 1,
    INTR_HTFIE = 2,
    INTR_ERRIE = 3
};

enum class Transfer_Direction : uint8_t {
    P2M,
    M2P
};

enum class Increase_Mode : uint8_t {
    INCREASE_DISABLE,
    INCREASE_ENABLE
};

enum class Bit_Width : uint8_t {
    WIDTH_8BIT,
    WIDTH_16BIT,
    WIDTH_32BIT
};

enum class Channel_Priority : uint8_t {
    LOW_PRIORITY,
    MEDIUM_PRIORITY,
    HIGH_PRIORITY,
    ULTRA_HIGH_PRIORITY
};

enum class M2M_Mode : uint8_t {
    M2M_MODE_DISABLE,
    M2M_MODE_ENABLE
};

enum class Data_Type : uint8_t {
    MEMORY_ADDRESS,
    PERIPHERAL_ADDRESS
};

enum class DMA_Error_Type : uint8_t {
    OK,
    INVALID_DMA,
    INVALID_CHANNEL,
    INVALID_OPERATION,
    INITIALIZATION_FAILED,
    INVALID_SELECTION
};


///////////////////////////// STRUCTURES /////////////////////////////

struct DMA_Clock_Config {
    rcu::RCU_PCLK clock_reg;
};

static inline constexpr std::array<DMA_Clock_Config, 2> DMA_pclk_index {{
    {rcu::RCU_PCLK::PCLK_DMA0},
    {rcu::RCU_PCLK::PCLK_DMA1}
}};

struct DMA_Config {
    uint32_t count;
    uint32_t memory_address;
    uint32_t peripheral_address;
    Bit_Width peripheral_bit_width;
    Bit_Width memory_bit_width;
    Increase_Mode peripheral_increase;
    Increase_Mode memory_increase;
    Channel_Priority channel_priority;
    Transfer_Direction direction;
    bool circular_mode;
    bool memory_to_memory;
};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t Lower16BitMask = 0x0000FFFFU;


///////////////////////////// INITIALIZATION DEFAULTS /////////////////////////////

static inline const DMA_Config default_config = {
    0U,
    0U,
    0U,
    Bit_Width::WIDTH_8BIT,
    Bit_Width::WIDTH_8BIT,
    Increase_Mode::INCREASE_DISABLE,
    Increase_Mode::INCREASE_DISABLE,
    Channel_Priority::LOW_PRIORITY,
    Transfer_Direction::P2M,
    false,
    false
};


} // namespace dma
