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

#include <cstdlib>
#include <cstdint>

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
    INTRS,
    INTHS,
    INTFS,
    INTREN,
    INTHEN,
    INTFEN,
    FFEPT
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
    INTR_FLAG_RISING,
    INTR_FLAG_LEVEL,
    INTR_FLAG_FALLING
};

enum class Interrupt_Type : uint8_t {
    INTR_RISING_EN = 3U,
    INTR_LEVEL_EN = 4U,
    INTR_FALLING_EN = 5U
};

///////////////////////////// STRUCTURES /////////////////////////////

struct NOR_SRAM_Timing {
    uint32_t bus_latency;
    uint32_t async_dst;             // Data setup time
    uint32_t async_aht;             // Address hold time
    uint32_t async_ast;             // Address setup time
    Async_Mode async_access : 2;
    Data_Latency sync_latency : 4;
    Sync_Divider divider : 4;
};

struct NOR_SRAM_Config {
    const NOR_SRAM_Timing* rw_timing;
    const NOR_SRAM_Timing* write_timing;
    Bus_Width width : 1;
    Block_Number block : 2;
    Write_Mode mode : 1;
    NWAIT_Active nwait_active : 1;
    Signal_Polarity polarity : 1;
    Memory_Type type : 2;
    bool async_wait : 1;
    bool extended_mode : 1;
    bool nwait_signal : 1;
    bool memory_write : 1;
    bool wrap : 1;
    bool burst : 1;
    bool address_mux : 1;
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
    Bus_Width databus_width : 1;
    NPC_Block npc_block : 2;
    ECC_Size ecc_size : 3;
    HCLK_Delay ctr_latency : 4;
    HCLK_Delay atr_latency : 4;
    bool ecc : 1;
    bool wait : 1;
};

struct PCCARD_Config {
    const NPC_Timing* common_timing;
    const NPC_Timing* attribute_timing;
    const NPC_Timing* io_timing;
    HCLK_Delay ctr_latency : 4;
    HCLK_Delay atr_latency : 4;
    bool wait : 1;
};

///////////////////////////// CONSTANTS /////////////////////////////

inline constexpr uint32_t NSRAM_Block_Offset = 0x08U;
inline constexpr uint32_t NPC_Block_Offset = 0x20U;

inline constexpr uint32_t SNCTL_Block0_Reset = 0x0000'30DBU;
inline constexpr uint32_t SNCTL_Block1_2_3_Reset = 0x0000'30D2U;
inline constexpr uint32_t Common_Reset = 0x0FFF'FFFFU;
inline constexpr uint32_t NPCTL_Block1_2_Reset = 0x0000'0018U;
inline constexpr uint32_t NPINTEN_Block1_2_Reset = 0x0000'0042U;
inline constexpr uint32_t NPCTL_Block3_Reset = 0x0000'0018U;
inline constexpr uint32_t NPINTEN_Block3_Reset = 0x0000'0043U;
inline constexpr uint32_t Common_Block3_Reset = 0xFCFC'FCFCU;

///////////////////////////// INITIALIZATION DEFAULTS /////////////////////////////

inline constexpr NOR_SRAM_Timing rw_timing_default = {
    .bus_latency = 0xFU,
    .async_dst = 0xFFU,
    .async_aht = 0xFU,
    .async_ast = 0xFU,
    .async_access = Async_Mode::MODE_A,
    .sync_latency = Data_Latency::LATENCY_17_CLK,
    .divider = Sync_Divider::DIV16
};

inline constexpr NOR_SRAM_Timing write_timing_default = {
    .bus_latency = 0xFU,
    .async_dst = 0xFFU,
    .async_aht = 0xFU,
    .async_ast = 0xFU,
    .async_access = Async_Mode::MODE_A,
    .sync_latency = Data_Latency::LATENCY_2_CLK,
    .divider = Sync_Divider::NONE
};

inline constexpr NOR_SRAM_Config nor_sram_default_config = {
    .rw_timing = &rw_timing_default,
    .write_timing = &write_timing_default,
    .width = Bus_Width::WIDTH_8BITS,
    .block = Block_Number::BLOCK0,
    .mode = Write_Mode::ASYNC,
    .nwait_active = NWAIT_Active::BEFORE,
    .polarity = Signal_Polarity::LOW,
    .type = Memory_Type::SRAM,
    .async_wait = false,
    .extended_mode = false,
    .nwait_signal = true,
    .memory_write = true,
    .wrap = false,
    .burst = false,
    .address_mux = true
};

inline constexpr NPC_Timing common_timing_default = {
    .dbhzt = 0xFCU,
    .ht = 0xFCU,
    .wt = 0xFCU,
    .st = 0xFCU
};

inline constexpr NAND_Config nand_default_config = {
    .common_timing = &common_timing_default,
    .attribute_timing = &common_timing_default,
    .databus_width = Bus_Width::WIDTH_8BITS,
    .npc_block = NPC_Block::NAND_BLOCK1,
    .ecc_size = ECC_Size::BYTES_256,
    .ctr_latency = HCLK_Delay::DELAY_1_HCLK,
    .atr_latency = HCLK_Delay::DELAY_1_HCLK,
    .ecc = false,
    .wait = false
};

inline constexpr NPC_Timing io_timing_default = {
    .dbhzt = 0xFCU,
    .ht = 0xFCU,
    .wt = 0xFCU,
    .st = 0xFCU
};

inline constexpr PCCARD_Config pccard_default_config = {
    .common_timing = &common_timing_default,
    .attribute_timing = &common_timing_default,
    .io_timing = &io_timing_default,
    .ctr_latency = HCLK_Delay::DELAY_1_HCLK,
    .atr_latency = HCLK_Delay::DELAY_1_HCLK,
    .wait = false
};

} // namespace exmc
