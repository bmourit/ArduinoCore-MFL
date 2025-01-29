//
// MFL gd32f30x CORTEX peripheral register access in C++
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

namespace cortex {


///////////////////////////// ADDRESS MAPS /////////////////////////////

static inline constexpr uintptr_t NVIC_vectorTable[] = {
    0x20000000,
    0x08000000
};

static inline constexpr uint32_t NVIC_priorityGroup[] = {
    0x00000300U,
    0x00000400U,
    0x00000500U,
    0x00000600U,
    0x00000700U
};


///////////////////////////// ENUMS /////////////////////////////

enum class Priority_Group : uint8_t {
    PRIO_GROUP_PRE4SUB0 = 3U,
    PRIO_GROUP_PRE3SUB1 = 4U,
    PRIO_GROUP_PRE2SUB2 = 5U,
    PRIO_GROUP_PRE1SUB3 = 6U,
    PRIO_GROUP_PRE0SUB4 = 7U
};

enum class Vector_Table : uint8_t {
    NVIC_VECTTAB_RAM,
    NVIC_VECTTAB_FLASH
};

enum class Low_Power_Mode : uint8_t {
    SLEEP_ON_EXIT = 1U,
    SLEEP_DEEP = 2U,
    SEV_ON_PEND = 4U
};

enum class Systick_Source : uint8_t {
    SYSTICK_SOURCE_HCLK,
    SYSTICK_SOURCE_HCLK_DIV8
};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t VectorTableSram = 0x20000000U;
static inline constexpr uint32_t VectorTableFlash = 0x08000000U;
static inline constexpr uint32_t SysTickSourceHClkDiv8 = 0xFFFFFFFBU;
static inline constexpr uint32_t SysTickSourceHClk = 0x00000004U;
static inline constexpr uint32_t VectorTableOffsetMask = 0x1FFFFF80U;
static inline constexpr uint32_t AIRCRRegisterMask = 0x00000700U;
static inline constexpr uint32_t VectKeyMask = 0x05FA0000U;
static inline constexpr uint32_t Pre0Sub4 = 0x00000700U;
static inline constexpr uint32_t Pre1Sub3 = 0x00000600U;
static inline constexpr uint32_t Pre2Sub2 = 0x00000500U;
static inline constexpr uint32_t Pre3Sub1 = 0x00000400U;
static inline constexpr uint32_t Pre4Sub0 = 0x00000300U;
static inline constexpr uint32_t SysTickCountFlagMask = 0x00010000U;


} // namespace cortex
