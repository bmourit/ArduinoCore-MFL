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

#include <cstdlib>
#include <cstdint>

namespace cortex {

///////////////////////////// ADDRESS MAPS /////////////////////////////

inline constexpr uintptr_t NVIC_vectorTable[] = {
    0x2000'0000,
    0x0800'0000
};

inline constexpr uint32_t NVIC_priorityGroup[] = {
    0x300U,
    0x400U,
    0x500U,
    0x600U,
    0x700U
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

inline constexpr uint32_t VectorTableSram = 0x2000'0000U;
inline constexpr uint32_t VectorTableFlash = 0x0800'0000U;
inline constexpr uint32_t SysTickSourceHClkDiv8 = 0xFFFF'FFFBU;
inline constexpr uint32_t SysTickSourceHClk = 0x4U;
inline constexpr uint32_t VectorTableOffsetMask = 0x1FFF'FF80U;
inline constexpr uint32_t AIRCRRegisterMask = 0x700U;
inline constexpr uint32_t VectKeyMask = 0x05FA'0000U;
inline constexpr uint32_t Pre0Sub4 = 0x700U;
inline constexpr uint32_t Pre1Sub3 = 0x600U;
inline constexpr uint32_t Pre2Sub2 = 0x500U;
inline constexpr uint32_t Pre3Sub1 = 0x400U;
inline constexpr uint32_t Pre4Sub0 = 0x300U;
inline constexpr uint32_t SysTickCountFlagMask = 0x0001'0000U;

} // namespace cortex
