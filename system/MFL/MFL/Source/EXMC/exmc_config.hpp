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

#include <stdlib.h>
#include <stdint.h>

#include "CONFIG.hpp"

namespace exmc {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class EXMC_Regs : uint32_t {
    SNCTL0 = 0x00U,
    SNTCFG0 = 0x04U,
    SNCTL1 = 0x08U,
    SNTCFG1 = 0x0CU,
    SNCTL2 = 0x10U,
    SNTCFG2 = 0x14U,
    SNCTL3 = 0x18U,
    SNTCFG3 = 0x1CU,
    NPCTL1 = 0x60U,
    NPINTEN1 = 0x64U,
    NPCTCFG1 = 0x68U,
    NPATCFG1 = 0x6CU,
    NECC1 = 0x74U,
    NPCTL2 = 0x80U,
    NPINTEN2 = 0x84U,
    NPCTCFG2 = 0x88U,
    NPATCFG2 = 0x8CU,
    NECC2 = 0x94U,
    NPCTL3 = 0xA0U,
    NPINTEN3 = 0xA4U,
    NPCTCFG3 = 0xA8U,
    NPATCFG3 = 0xACU,
    PIOTCFG3 = 0xB0U,
    SNWTCFG0 = 0x104U,
    SNWTCFG1 = 0x10CU,
    SNWTCFG2 = 0x114U,
    SNWTCFG3 = 0x11CU
};

enum class EXMC_Base_Regs : uint32_t {
    SNCTL_BASE = 0x00U,
    SNTCFG_BASE = 0x04U,
    NPCTL_BASE = 0x60U,
    NPINTEN_BASE = 0x64U,
    NPCTCFG_BASE = 0x68U,
    NPATCFG_BASE = 0x6CU,
    NECC_BASE = 0x74U,
    SNWTCFG_BASE = 0x104U
};

enum class SNCTLX_Bits : uint32_t {
    NRBKEN = 0U,
    NRMUX = 1U,
    NRTP = REG_BIT_DEF(2, 3),
    NRW = REG_BIT_DEF(4, 5),
    NREN = 6U,
    SBRSTEN = 8U,
    NRWTPOL = 9U,
    WRAPEN = 10U,
    NRWTCFG = 11U,
    WREN = 12U,
    NRWTEN = 13U,
    EXMODEN = 14U,
    ASYNCWAIT = 15U,
    CPS = REG_BIT_DEF(16, 18),
    SYNCWR = 19U
};

enum class SNTCFGX_Bits : uint32_t {
    ASET = REG_BIT_DEF(0, 3),
    AHLD = REG_BIT_DEF(4, 7),
    DSET = REG_BIT_DEF(8, 15),
    BUSLAT = REG_BIT_DEF(16, 19),
    CKDIV = REG_BIT_DEF(20, 23),
    DLAT = REG_BIT_DEF(24, 27),
    ASYNCMOD = REG_BIT_DEF(28, 29)
};

enum class SNWTCFGX_Bits : uint32_t {
    WASET = REG_BIT_DEF(0, 3),
    WAHLD = REG_BIT_DEF(4, 7),
    WDSET = REG_BIT_DEF(8, 15),
    WBUSLAT = REG_BIT_DEF(16, 19),
    WASYNCMOD = REG_BIT_DEF(28, 29)
};

enum class NPCTLX_Bits : uint32_t {
    NDWTEN = 1U,
    NDBKEN = 2U,
    NDTP = 3U,
    NDW = REG_BIT_DEF(4, 5),
    ECCEN = 6U,
    CTR = REG_BIT_DEF(9, 12),
    ATR = REG_BIT_DEF(13, 16),
    ECCSZ = REG_BIT_DEF(17, 19)
};

enum class NPINTENX_Bits : uint8_t {
    INTRS = 0U,
    INTHS = 1U,
    INTFS = 2U,
    INTREN = 3U,
    INTHEN = 4U,
    INTFEN = 5U,
    FFEPT = 6U
};

enum class NPCTCFGX_Bits : uint32_t {
    COMSET = REG_BIT_DEF(0, 7),
    COMWAIT = REG_BIT_DEF(8, 15),
    COMHLD = REG_BIT_DEF(16, 23),
    COMHIZ = REG_BIT_DEF(24, 31)
};

enum class NPATCFGX_Bits : uint32_t {
    ATTSET = REG_BIT_DEF(0, 7),
    ATTWAIT = REG_BIT_DEF(8, 15),
    ATTHLD = REG_BIT_DEF(16, 23),
    ATTHIZ = REG_BIT_DEF(24, 31)
};

enum class PIOTCFG3_Bits : uint32_t {
    IOSET = REG_BIT_DEF(0, 7),
    IOWAIT = REG_BIT_DEF(8, 15),
    IOHLD = REG_BIT_DEF(16, 23),
    IOHIZ = REG_BIT_DEF(24, 31)
};

enum class NECCX_Bits : uint32_t {
    ECC = REG_BIT_DEF(0, 31)
};


///////////////////////////// ENUMS /////////////////////////////

enum class Page_Size : uint8_t {
    AUTO_SPLIT,
    BYTES_128,
    BYTES_256,
    BYTES_512,
    BYTES_1024
};

enum class Bus_Width : uint8_t {
    WIDTH_8BITS,
    WIDTH_16BITS
};

enum class Memory_Type : uint8_t {
    SRAM,
    PSRAM,
    NOR
};

enum class Async_Mode : uint8_t {
    MODE_A,
    MODE_B,
    MODE_C,
    MODE_D
};

enum class Data_Latency : uint8_t {
    LATENCY_2_CLK,
    LATENCY_3_CLK,
    LATENCY_4_CLK,
    LATENCY_5_CLK,
    LATENCY_6_CLK,
    LATENCY_7_CLK,
    LATENCY_8_CLK,
    LATENCY_9_CLK,
    LATENCY_10_CLK,
    LATENCY_11_CLK,
    LATENCY_12_CLK,
    LATENCY_13_CLK,
    LATENCY_14_CLK,
    LATENCY_15_CLK,
    LATENCY_16_CLK,
    LATENCY_17_CLK
};

enum class Sync_Divider : uint8_t {
    NONE,
    DIV2,
    DIV3,
    DIV4,
    DIV5,
    DIV6,
    DIV7,
    DIV8,
    DIV9,
    DIV10,
    DIV11,
    DIV12,
    DIV13,
    DIV14,
    DIV15,
    DIV16
};

enum class ECC_Size : uint8_t {
    BYTES_256,
    BYTES_512,
    BYTES_1024,
    BYTES_2048,
    BYTES_4096,
    BYTES_8192
};

enum class HCLK_Delay : uint8_t {
    DELAY_1_HCLK,
    DELAY_2_HCLK,
    DELAY_3_HCLK,
    DELAY_4_HCLK,
    DELAY_5_HCLK,
    DELAY_6_HCLK,
    DELAY_7_HCLK,
    DELAY_8_HCLK,
    DELAY_9_HCLK,
    DELAY_10_HCLK,
    DELAY_11_HCLK,
    DELAY_12_HCLK,
    DELAY_13_HCLK,
    DELAY_14_HCLK,
    DELAY_15_HCLK,
    DELAY_16_HCLK
};

enum class Block_Number : uint8_t {
    BLOCK0,
    BLOCK1,
    BLOCK2,
    BLOCK3
};

enum class Write_Mode : uint8_t {
    ASYNC,
    SYNC
};

enum class NWAIT_Active : uint8_t {
    BEFORE,
    DURING
};

enum Signal_Polarity : uint8_t {
    LOW,
    HIGH
};

enum class NPC_Block : uint8_t {
    NAND_BLOCK1,
    NAND_BLOCK2,
    PCCARD_BLOCK3
};

enum class Status_Flags : uint32_t {
    FLAG_RISING = 0U,
    FLAG_LEVEL = 1U,
    FLAG_FALLING = 2U,
    FLAG_FIFOE = 6U
};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_RISING = 0U,
    INTR_FLAG_LEVEL = 1U,
    INTR_FLAG_FALLING = 2U
};

enum class Interrupt_Type : uint8_t {
    INTR_RISING_EN = 3U,
    INTR_LEVEL_EN = 4U,
    INTR_FALLING_EN = 5U
};


///////////////////////////// STRUCTURES /////////////////////////////

struct NOR_SRAM_Timing {
    uint32_t bus_latency;
    uint32_t async_dst;     // Data setup time
    uint32_t async_aht;     // Address hold time
    uint32_t async_ast;     // Address setup time
    Async_Mode async_access;
    Data_Latency sync_latency;
    Sync_Divider divider;
};

struct NOR_SRAM_Config {
    const NOR_SRAM_Timing* rw_timing;
    const NOR_SRAM_Timing* write_timing;
    Bus_Width width;
    Block_Number block;
    Write_Mode mode;
    NWAIT_Active nwait_active;
    Signal_Polarity polarity;
    Memory_Type type;
    bool async_wait;
    bool extended_mode;
    bool nwait_signal;
    bool memory_write;
    bool wrap;
    bool burst;
    bool address_mux;
};

struct NPC_Timing {
    uint32_t dbhzt; // Data bus Hi z time
    uint32_t ht;    // Hold time
    uint32_t wt;    // Wait time
    uint32_t st;    // Setup time
};

struct NAND_Config {
    const NPC_Timing* common_timing;
    const NPC_Timing* attribute_timing;
    Bus_Width databus_width;
    NPC_Block npc_block;
    ECC_Size ecc_size;
    HCLK_Delay ctr_latency;
    HCLK_Delay atr_latency;
    bool ecc;
    bool wait;
};

struct PCCARD_Config {
    const NPC_Timing* common_timing;
    const NPC_Timing* attribute_timing;
    const NPC_Timing* io_timing;
    HCLK_Delay ctr_latency;
    HCLK_Delay atr_latency;
    bool wait;
};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t NSRAM_Block_Offset = 0x08U;
static inline constexpr uint32_t NPC_Block_Offset = 0x20U;

static inline constexpr uint32_t SNCTL_Block0_Reset = 0x000030DBU;
static inline constexpr uint32_t SNCTL_Block1_2_3_Reset = 0x000030D2U;
static inline constexpr uint32_t Common_Reset = 0x0FFFFFFFU;
static inline constexpr uint32_t NPCTL_Block1_2_Reset = 0x00000018U;
static inline constexpr uint32_t NPINTEN_Block1_2_Reset = 0x00000042U;
static inline constexpr uint32_t NPCTL_Block3_Reset = 0x00000018U;
static inline constexpr uint32_t NPINTEN_Block3_Reset = 0x00000043U;
static inline constexpr uint32_t Common_Block3_Reset = 0xFCFCFCFCU;


///////////////////////////// INITIALIZATION DEFAULTS /////////////////////////////

static inline const NOR_SRAM_Timing rw_timing_default = {
    0xFU,
    0xFFU,
    0xFU,
    0xFU,
    Async_Mode::MODE_A,
    Data_Latency::LATENCY_17_CLK,
    Sync_Divider::DIV16
};

static inline const NOR_SRAM_Timing write_timing_default = {
    0xFU,
    0xFFU,
    0xFU,
    0xFU,
    Async_Mode::MODE_A,
    Data_Latency::LATENCY_2_CLK,
    Sync_Divider::NONE
};

static inline const NOR_SRAM_Config nor_sram_default_config = {
    &rw_timing_default,
    &write_timing_default,
    Bus_Width::WIDTH_8BITS,
    Block_Number::BLOCK0,
    Write_Mode::ASYNC,
    NWAIT_Active::BEFORE,
    Signal_Polarity::LOW,
    Memory_Type::SRAM,
    false,
    false,
    true,
    true,
    false,
    false,
    true
};

static inline const NPC_Timing common_timing_default = {
    0xFCU,
    0xFCU,
    0xFCU,
    0xFCU
};

static inline const NAND_Config nand_default_config = {
    &common_timing_default,
    &common_timing_default,
    Bus_Width::WIDTH_8BITS,
    NPC_Block::NAND_BLOCK1,
    ECC_Size::BYTES_256,
    HCLK_Delay::DELAY_1_HCLK,
    HCLK_Delay::DELAY_1_HCLK,
    false,
    false
};

static inline const NPC_Timing io_timing_default = {
    0xFCU,
    0xFCU,
    0xFCU,
    0xFCU
};

static inline const PCCARD_Config pccard_default_config = {
    &common_timing_default,
    &common_timing_default,
    &io_timing_default,
    HCLK_Delay::DELAY_1_HCLK,
    HCLK_Delay::DELAY_1_HCLK,
    false
};

} // namespace exmc
